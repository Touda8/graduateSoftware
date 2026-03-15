#include "reconstruction/ReconstructionPipeline.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <chrono>

namespace tp {

namespace {
// 计时辅助宏
using SteadyClock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

inline long long elapsedMs(const SteadyClock::time_point& start) {
    return std::chrono::duration_cast<Ms>(SteadyClock::now() - start).count();
}
} // namespace

void ReconstructionPipeline::setCalibData(const CalibData& calib) {
    calibData_ = calib;
    Logger::instance().info("ReconstructionPipeline: calibData set");
}

void ReconstructionPipeline::setParams(const ReconParams& params) {
    params_ = params;
    Logger::instance().info("ReconstructionPipeline: params set");
}

// run 将在此后插入

ReconstructionPipeline::CloudPtr
ReconstructionPipeline::run(const std::vector<cv::Mat>& images)
{
    Logger::instance().info("Pipeline: starting reconstruction");

    if (images.empty()) {
        throw ReconstructException("Pipeline: no input images");
    }

    const int steps = params_.shift1;
    const int total = steps * 3; // 三频，每频 steps 张

    if (static_cast<int>(images.size()) < total) {
        throw ReconstructException("Pipeline: need >= " +
            std::to_string(total) + " images, got " +
            std::to_string(images.size()));
    }

    // 分频段提取图像
    std::vector<cv::Mat> imgs1(images.begin(), images.begin() + steps);
    std::vector<cv::Mat> imgs2(images.begin() + steps,
                               images.begin() + 2 * steps);
    std::vector<cv::Mat> imgs3(images.begin() + 2 * steps,
                               images.begin() + 3 * steps);

    // 计算三频包裹相位
    cv::Mat phi1 = decoder_.calcPhaseMatrix(imgs1, steps);
    cv::Mat phi2 = decoder_.calcPhaseMatrix(imgs2, steps);
    cv::Mat phi3 = decoder_.calcPhaseMatrix(imgs3, steps);

    // 三频解包裹
    cv::Mat absPhase = decoder_.multiFreqUnwrap(
        phi1, phi2, phi3,
        params_.freq1, params_.freq2, params_.freq3);

    // 计算高度图
    cv::Mat heightMap = heightCalc_.calcPlaneHeight(absPhase, calibData_);

    // 构建点云
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int r = 0; r < heightMap.rows; ++r) {
        for (int c = 0; c < heightMap.cols; ++c) {
            float z = heightMap.at<float>(r, c);
            if (z == 0.0f) continue;
            float x = calibData_.X.empty() ? static_cast<float>(c)
                     : calibData_.X.at<float>(r, c);
            float y = calibData_.Y.empty() ? static_cast<float>(r)
                     : calibData_.Y.at<float>(r, c);
            cloud->push_back({x, y, z});
        }
    }

    if (cloud->size() < 100) {
        throw ReconstructException("Pipeline: cloud has < 100 points ("
            + std::to_string(cloud->size()) + ")");
    }

    Logger::instance().info("Pipeline: reconstruction complete, "
        + std::to_string(cloud->size()) + " points");
    return cloud;
}

// 单投影仪相位解码结果
struct DecodeResult {
    cv::Mat Z;        // 高度图
    cv::Mat absPhase; // 绝对相位图
    cv::Mat modContrast; // 调制度对比度（用于SNR加权融合）
};

// 功能：单投影仪相位解码→高度图，输入：images/allK_i/params，返回：DecodeResult
static DecodeResult decodeAndHeight(
    PhaseDecoder& decoder,
    const std::vector<cv::Mat>& images,
    const cv::Mat& allK_i,
    int imgRows, int imgCols,
    const ReconParams& params)
{
    const int steps = params.shift1;
    const int total = steps * 3;
    if (static_cast<int>(images.size()) < total) {
        throw ReconstructException("runDual: need >= "
            + std::to_string(total) + " images, got "
            + std::to_string(images.size()));
    }

    // 预处理：高斯滤波（匹配 MATLAB fspecial('gaussian',[9,9],3)）
    auto tBlur = SteadyClock::now();
    auto blurImages = [&](int startIdx) {
        std::vector<cv::Mat> out;
        out.reserve(steps);
        for (int i = 0; i < steps; ++i) {
            cv::Mat blurred;
            cv::GaussianBlur(images[startIdx + i], blurred,
                cv::Size(9, 9), 3.0, 3.0, cv::BORDER_REPLICATE);
            out.push_back(blurred);
        }
        return out;
    };
    std::vector<cv::Mat> f1Imgs = blurImages(0);
    std::vector<cv::Mat> f2Imgs = blurImages(steps);
    std::vector<cv::Mat> f3Imgs = blurImages(2 * steps);
    Logger::instance().info("[Timing] GaussianBlur x" + std::to_string(total) +
        ": " + std::to_string(elapsedMs(tBlur)) + " ms");

    auto tPhase = SteadyClock::now();
    cv::Mat phi1 = decoder.calcPhaseMatrix(f1Imgs, steps);
    cv::Mat phi2 = decoder.calcPhaseMatrix(f2Imgs, steps);
    cv::Mat phi3 = decoder.calcPhaseMatrix(f3Imgs, steps);
    Logger::instance().info("[Timing] calcPhaseMatrix x3: " +
        std::to_string(elapsedMs(tPhase)) + " ms");

    auto tUnwrap = SteadyClock::now();
    cv::Mat absPhase = decoder.multiFreqUnwrap(
        phi1, phi2, phi3,
        params.freq1, params.freq2, params.freq3);
    Logger::instance().info("[Timing] multiFreqUnwrap: " +
        std::to_string(elapsedMs(tUnwrap)) + " ms");

    // 高度计算: Z = (phi*c1 + c3) / (c2*phi + 1)
    // allK_i 由 MATLAB reshape(allK_i_3D,[],3) 列主序生成，线性索引 = c*imgRows+r
    auto tHeight = SteadyClock::now();
    cv::Mat Z = cv::Mat::zeros(imgRows, imgCols, CV_64F);
    for (int r = 0; r < imgRows; ++r) {
        for (int c = 0; c < imgCols; ++c) {
            int idx = c * imgRows + r;
            double phiVal = absPhase.at<double>(r, c);
            double c1 = allK_i.at<double>(idx, 0);
            double c2 = allK_i.at<double>(idx, 1);
            double c3 = allK_i.at<double>(idx, 2);
            double denom = c2 * phiVal + 1.0;
            if (std::abs(denom) > 1e-10)
                Z.at<double>(r, c) = (phiVal * c1 + c3) / denom;
        }
    }
    Logger::instance().info("[Timing] heightCalc: " +
        std::to_string(elapsedMs(tHeight)) + " ms");

    DecodeResult dr;
    dr.Z = Z;
    dr.absPhase = absPhase;
    // 调制度对比度（使用频率1的图像，匹配MATLAB PhaseQualityMask）
    auto tMod = SteadyClock::now();
    dr.modContrast = decoder.calcModulationContrast(f1Imgs, steps);
    Logger::instance().info("[Timing] calcModContrast: " +
        std::to_string(elapsedMs(tMod)) + " ms");
    return dr;
}

// 功能：双投影仪重建，输入：两路图像/彩色图/标定/参数，返回：DualReconResult
DualReconResult ReconstructionPipeline::runDual(
    const std::vector<cv::Mat>& images1,
    const std::vector<cv::Mat>& images2,
    const cv::Mat& colorImage,
    const DualCalibData& calib,
    const ReconParams& params)
{
    DualReconResult result;
    try {
        if (!calib.isValid) {
            throw ReconstructException("runDual: invalid calib data");
        }
        if (images1.empty() || images2.empty()) {
            throw ReconstructException("runDual: empty images");
        }
        if (colorImage.empty()) {
            Logger::instance().warn("runDual: colorImage is empty, using gray fallback");
        }

        const int rows = calib.imgRows;
        const int cols = calib.imgCols;

        auto tTotal = SteadyClock::now();

        // 两路投影仪各自解码+计算高度
        Logger::instance().info("[Timing] === Pro1 decode start ===");
        auto tDec1 = SteadyClock::now();
        auto dec1 = decodeAndHeight(
            decoder_, images1, calib.allK_i1, rows, cols, params);
        Logger::instance().info("[Timing] Pro1 decode total: " +
            std::to_string(elapsedMs(tDec1)) + " ms");

        Logger::instance().info("[Timing] === Pro2 decode start ===");
        auto tDec2 = SteadyClock::now();
        auto dec2 = decodeAndHeight(
            decoder_, images2, calib.allK_i2, rows, cols, params);
        Logger::instance().info("[Timing] Pro2 decode total: " +
            std::to_string(elapsedMs(tDec2)) + " ms");
        cv::Mat Z1 = dec1.Z;
        cv::Mat Z2 = dec2.Z;
        result.absPhase1 = dec1.absPhase;
        result.absPhase2 = dec2.absPhase;

        Logger::instance().info("runDual: height maps computed");

        // SNR加权融合（匹配MATLAB: mix_Z = (Hb1*Z1 + Hb2*Z2) / (Hb1+Hb2)）
        auto tFusion = SteadyClock::now();
        cv::Mat snr1 = dec1.modContrast;
        cv::Mat snr2 = dec2.modContrast;
        cv::Mat mixZ = cv::Mat::zeros(rows, cols, CV_64F);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                double z1 = Z1.at<double>(r, c);
                double z2 = Z2.at<double>(r, c);
                double s1 = snr1.at<double>(r, c);
                double s2 = snr2.at<double>(r, c);
                double sumS = s1 + s2;
                if (sumS > 1e-10)
                    mixZ.at<double>(r, c) = (s1 * z1 + s2 * z2) / sumS;
                else if (z1 != 0.0)
                    mixZ.at<double>(r, c) = z1;
                else
                    mixZ.at<double>(r, c) = z2;
            }
        }
        Logger::instance().info("[Timing] SNR fusion: " +
            std::to_string(elapsedMs(tFusion)) + " ms");

        // 防御：验证标定矩阵尺寸一致
        if (calib.X.rows != rows || calib.X.cols != cols) {
            throw ReconstructException("runDual: X size mismatch: "
                + std::to_string(calib.X.rows) + "x" + std::to_string(calib.X.cols)
                + " vs expected " + std::to_string(rows) + "x" + std::to_string(cols));
        }
        if (calib.Y.rows != rows || calib.Y.cols != cols) {
            throw ReconstructException("runDual: Y size mismatch");
        }
        // colorImage 尺寸不一致时缩放
        cv::Mat colorResized = colorImage;
        if (!colorImage.empty() &&
            (colorImage.rows != rows || colorImage.cols != cols)) {
            Logger::instance().warn("runDual: colorImage size mismatch, resizing");
            cv::resize(colorImage, colorResized, cv::Size(cols, rows),
                       0, 0, cv::INTER_LINEAR);
        }

        // 构建彩色点云
        auto tCloud = SteadyClock::now();
        auto cloud = std::make_shared<
            pcl::PointCloud<pcl::PointXYZRGB>>();
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                double zv = mixZ.at<double>(r, c);
                if (zv == 0.0) continue;
                pcl::PointXYZRGB pt;
                pt.x = static_cast<float>(
                    calib.X.at<double>(r, c));
                pt.y = static_cast<float>(
                    calib.Y.at<double>(r, c));
                pt.z = static_cast<float>(zv);
                uint8_t gray = 128;
                if (!colorResized.empty()
                    && r < colorResized.rows
                    && c < colorResized.cols)
                    gray = colorResized.at<uchar>(r, c);
                pt.r = gray; pt.g = gray; pt.b = gray;
                cloud->push_back(pt);
            }
        }

        Logger::instance().info("[Timing] cloudBuild: " +
            std::to_string(elapsedMs(tCloud)) + " ms");
        Logger::instance().info("runDual: cloud size="
            + std::to_string(cloud->size()));
        Logger::instance().info("[Timing] runDual TOTAL: " +
            std::to_string(elapsedMs(tTotal)) + " ms");
        result.cloud = cloud;
        result.heightMap = mixZ;
        result.success = true;
    } catch (const std::exception& e) {
        result.success = false;
        result.errorMsg = e.what();
        Logger::instance().warn(
            std::string("runDual failed: ") + e.what());
    }
    return result;
}

} // namespace tp