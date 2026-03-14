#define _USE_MATH_DEFINES
#include "PhaseDecoder.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <stdexcept>

namespace tp {

// 功能：N步相移包裹相位计算，输入：imgs(N张图), steps，返回：包裹相位[-π,π]
cv::Mat PhaseDecoder::calcPhaseMatrix(
    const std::vector<cv::Mat>& imgs, int steps)
{
    if (imgs.empty()) {
        throw tp::ImageLoadException("calcPhaseMatrix: imgs is empty");
    }
    if (steps < 3 || static_cast<int>(imgs.size()) < steps) {
        throw tp::ImageLoadException("calcPhaseMatrix: invalid steps or insufficient images");
    }
    const int rows = imgs[0].rows;
    const int cols = imgs[0].cols;

    if (steps == 4) {
        // 四步相移优化路径: atan2(I4-I2, I1-I3)
        cv::Mat I1, I2, I3, I4;
        imgs[0].convertTo(I1, CV_64F);
        imgs[1].convertTo(I2, CV_64F);
        imgs[2].convertTo(I3, CV_64F);
        imgs[3].convertTo(I4, CV_64F);
        cv::Mat num = I4 - I2;
        cv::Mat den = I1 - I3;
        cv::Mat phase(rows, cols, CV_64F);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                phase.at<double>(r, c) =
                    std::atan2(num.at<double>(r, c), den.at<double>(r, c));
            }
        }
        return phase;
    }

    // 通用N步相移
    cv::Mat sinSum = cv::Mat::zeros(rows, cols, CV_64F);
    cv::Mat cosSum = cv::Mat::zeros(rows, cols, CV_64F);
    for (int n = 0; n < steps; ++n) {
        const double angle = -2.0 * M_PI * n / steps;
        cv::Mat img64;
        imgs[n].convertTo(img64, CV_64F);
        sinSum += img64 * std::sin(angle);
        cosSum += img64 * std::cos(angle);
    }
    sinSum = -sinSum; // negate for atan2 convention
    cv::Mat phase(rows, cols, CV_64F);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            phase.at<double>(r, c) =
                std::atan2(sinSum.at<double>(r, c), cosSum.at<double>(r, c));
        }
    }
    return phase;
}

// 功能：条纹调制度计算，输入：imgs(N张), steps，返回：调制度图
cv::Mat PhaseDecoder::calcFringeModulation(
    const std::vector<cv::Mat>& imgs, int steps)
{
    if (imgs.empty()) {
        throw tp::ImageLoadException("calcFringeModulation: imgs is empty");
    }
    if (steps < 3 || static_cast<int>(imgs.size()) < steps) {
        throw tp::ImageLoadException("calcFringeModulation: invalid steps");
    }
    const int rows = imgs[0].rows;
    const int cols = imgs[0].cols;

    cv::Mat sinPart = cv::Mat::zeros(rows, cols, CV_64F);
    cv::Mat cosPart = cv::Mat::zeros(rows, cols, CV_64F);
    constexpr double SCALE = 255.0;

    for (int i = 0; i < steps; ++i) {
        const double angle = 2.0 * M_PI * (i + 1) / steps;
        cv::Mat img64;
        imgs[i].convertTo(img64, CV_64F);
        sinPart += img64 * (SCALE * std::sin(angle));
        cosPart += img64 * (SCALE * std::cos(angle));
    }

    cv::Mat modulation;
    cv::magnitude(sinPart, cosPart, modulation);
    modulation *= (2.0 / steps);

    // E03: 低调制度像素占比过高时发出警告
    int totalPixels = modulation.rows * modulation.cols;
    if (totalPixels > 0) {
        int lowModCount = cv::countNonZero(modulation < 1.0);
        double lowRatio = static_cast<double>(lowModCount) / totalPixels;
        if (lowRatio > 0.8) {
            tp::Logger::instance().warn(
                "calcFringeModulation: low modulation ratio "
                + std::to_string(lowRatio));
        }
    }
    return modulation;
}

// 功能：调制度对比度计算，输入：imgs(N张), steps，返回：调制度/平均亮度
cv::Mat PhaseDecoder::calcModulationContrast(
    const std::vector<cv::Mat>& imgs, int steps)
{
    if (imgs.empty()) {
        throw tp::ImageLoadException("calcModulationContrast: imgs is empty");
    }
    if (steps < 3 || static_cast<int>(imgs.size()) < steps) {
        throw tp::ImageLoadException("calcModulationContrast: invalid steps");
    }
    const int rows = imgs[0].rows;
    const int cols = imgs[0].cols;
    constexpr double SCALE = 255.0;

    cv::Mat sinPart = cv::Mat::zeros(rows, cols, CV_64F);
    cv::Mat cosPart = cv::Mat::zeros(rows, cols, CV_64F);
    cv::Mat meanPic = cv::Mat::zeros(rows, cols, CV_64F);

    for (int i = 0; i < steps; ++i) {
        const double angle = 2.0 * M_PI * i / steps;
        cv::Mat img64;
        imgs[i].convertTo(img64, CV_64F);
        sinPart += img64 * (SCALE * std::sin(angle));
        cosPart += img64 * (SCALE * std::cos(angle));
        meanPic += img64 * SCALE;
    }
    meanPic /= steps;

    cv::Mat modulation;
    cv::magnitude(sinPart, cosPart, modulation);
    modulation *= (2.0 / steps);

    // 避免除零
    cv::Mat safeMean;
    cv::max(meanPic, 1e-10, safeMean);
    cv::Mat contrast;
    cv::divide(modulation, safeMean, contrast);
    return contrast;
}

// 功能：相位误差能量计算，输入：imgs, wrappedPhase, steps，返回：归一化误差能量图
cv::Mat PhaseDecoder::calcPhaseErrorEnergy(
    const std::vector<cv::Mat>& imgs,
    const cv::Mat& wrappedPhase, int steps)
{
    if (imgs.empty() || wrappedPhase.empty()) {
        throw tp::ImageLoadException("calcPhaseErrorEnergy: empty input");
    }
    if (steps < 3 || static_cast<int>(imgs.size()) < steps) {
        throw tp::ImageLoadException("calcPhaseErrorEnergy: invalid steps");
    }
    const int rows = imgs[0].rows;
    const int cols = imgs[0].cols;
    constexpr double SCALE = 255.0;

    // 调制度
    cv::Mat mod = calcFringeModulation(imgs, steps);
    cv::Mat safeMod;
    cv::max(mod, 1e-10, safeMod);

    // 平均亮度
    cv::Mat meanPic = cv::Mat::zeros(rows, cols, CV_64F);
    for (int i = 0; i < steps; ++i) {
        cv::Mat img64;
        imgs[i].convertTo(img64, CV_64F);
        meanPic += img64 * SCALE;
    }
    meanPic /= steps;

    // 误差能量累加
    cv::Mat errorEnergy = cv::Mat::zeros(rows, cols, CV_64F);
    for (int i = 0; i < steps; ++i) {
        cv::Mat img64;
        imgs[i].convertTo(img64, CV_64F);
        cv::Mat normalized = (img64 * SCALE - meanPic) / safeMod;
        double angle = 2.0 * M_PI * i / steps;
        cv::Mat ideal(rows, cols, CV_64F);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                ideal.at<double>(r, c) =
                    std::cos(wrappedPhase.at<double>(r, c) - angle);
            }
        }
        cv::Mat diff = normalized - ideal;
        errorEnergy += diff.mul(diff);
    }
    cv::sqrt(errorEnergy / steps, errorEnergy);
    return errorEnergy;
}

// 功能：时间相位展开，输入：keyPhase(粗), multiple, wrappedPhase(精)，返回：展开相位
cv::Mat PhaseDecoder::unwrapPhase(
    const cv::Mat& keyPhase, int multiple,
    const cv::Mat& wrappedPhase)
{
    if (keyPhase.empty() || wrappedPhase.empty()) {
        throw tp::PhaseDecodeException("unwrapPhase: empty input");
    }
    const int rows = keyPhase.rows;
    const int cols = keyPhase.cols;
    constexpr double TWO_PI = 2.0 * M_PI;

    cv::Mat result(rows, cols, CV_64F);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double key = keyPhase.at<double>(r, c);
            double wrapped = wrappedPhase.at<double>(r, c);
            double k = std::round((multiple * key - wrapped) / TWO_PI);
            result.at<double>(r, c) = wrapped + k * TWO_PI;
        }
    }

    // E04: 检查 NaN 并填充为 0，记录比例
    int nanCount = 0;
    int total = rows * cols;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (!std::isfinite(result.at<double>(r, c))) {
                result.at<double>(r, c) = 0.0;
                ++nanCount;
            }
        }
    }
    if (nanCount > 0) {
        tp::Logger::instance().info(
            "unwrapPhase: filled " + std::to_string(nanCount)
            + "/" + std::to_string(total) + " NaN pixels with 0");
    }
    return result;
}

// 功能：差频相位计算，输入：pha1, pha2 ∈ [-π,π]，返回：差频相位 ∈ [0, 2π)
cv::Mat PhaseDecoder::multiFreqDiff(
    const cv::Mat& pha1, const cv::Mat& pha2)
{
    constexpr double TWO_PI = 2.0 * M_PI;
    const int rows = pha1.rows;
    const int cols = pha1.cols;

    cv::Mat result(rows, cols, CV_64F);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double p1 = pha1.at<double>(r, c);
            double p2 = pha2.at<double>(r, c);
            double diff = p1 - p2;
            if (diff < 0.0) diff += TWO_PI;
            result.at<double>(r, c) = diff;
        }
    }
    return result;
}

// 功能：三频绝对相位展开，输入：phi1-3包裹相位, f1-3频率，返回：绝对相位
cv::Mat PhaseDecoder::multiFreqUnwrap(
    const cv::Mat& phi1, const cv::Mat& phi2, const cv::Mat& phi3,
    int f1, int f2, int f3)
{
    if (phi1.empty() || phi2.empty() || phi3.empty()) {
        throw tp::PhaseDecodeException("multiFreqUnwrap: empty input");
    }
    // 差频相位 (等效低频) — 匹配 MATLAB: Multifrequency_phase(PHDeformed_3, PHDeformed_2)
    cv::Mat pha12 = multiFreqDiff(phi1, phi2);
    cv::Mat pha23 = multiFreqDiff(phi3, phi2);
    cv::Mat pha123 = multiFreqDiff(pha12, pha23);

    // 等效频率
    int f12 = std::abs(f1 - f2);
    int f23 = std::abs(f2 - f3);
    int f123 = std::abs(f12 - f23);
    if (f123 == 0) f123 = 1; // 防零除

    // 逐级展开: pha123 → pha12 → phi1
    int mult12 = f12 / std::max(f123, 1);
    cv::Mat unwrapped12 = unwrapPhase(pha123, mult12, pha12);

    int mult1 = f1 / std::max(f12, 1);
    cv::Mat unwrapped1 = unwrapPhase(unwrapped12, mult1, phi1);
    return unwrapped1;
}

} // namespace tp
