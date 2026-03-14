#define _USE_MATH_DEFINES
#include "HeightCalculator.h"
#include "PhaseDecoder.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace tp {

// 功能：PCA平面拟合，输入：Mx3点云矩阵，返回：平面参数(A,B,C,D)
Eigen::Vector4d HeightCalculator::fitPlanePCA(const cv::Mat& points)
{
    if (points.empty() || points.cols != 3) {
        throw tp::ReconstructException("fitPlanePCA: points must be Mx3");
    }
    const int m = points.rows;

    // 计算质心
    Eigen::Vector3d centroid(0, 0, 0);
    for (int i = 0; i < m; ++i) {
        centroid(0) += points.at<double>(i, 0);
        centroid(1) += points.at<double>(i, 1);
        centroid(2) += points.at<double>(i, 2);
    }
    centroid /= m;

    // 协方差矩阵
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i = 0; i < m; ++i) {
        Eigen::Vector3d p(points.at<double>(i, 0) - centroid(0),
                          points.at<double>(i, 1) - centroid(1),
                          points.at<double>(i, 2) - centroid(2));
        cov += p * p.transpose();
    }
    cov /= (m - 1);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    // 最小特征值对应法向量
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    normal.normalize();

    double d = -normal.dot(centroid);
    double sign = (d >= 0) ? 1.0 : -1.0;
    return Eigen::Vector4d(normal(0) * sign, normal(1) * sign,
                           normal(2) * sign, d * sign);
}

// 功能：由绝对相位+标定系数计算高度图
cv::Mat HeightCalculator::calcPlaneHeight(
    const cv::Mat& absPhase, const CalibData& calib)
{
    if (absPhase.empty()) {
        throw tp::CalibException("calcPlaneHeight: absPhase is empty");
    }
    if (!calib.isValid || calib.allK.empty()) {
        throw tp::CalibException("calcPlaneHeight: invalid CalibData");
    }
    const int rows = absPhase.rows;
    const int cols = absPhase.cols;
    const int nCoeffs = calib.allK.cols;

    cv::Mat height = cv::Mat::zeros(rows, cols, CV_64F);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double phi = absPhase.at<double>(r, c);
            double h = 0.0;
            double phiPow = 1.0;
            // 多项式: h = K(n)*phi^0 + K(n-1)*phi^1 + ... + K(1)*phi^(n-1)
            for (int k = nCoeffs - 1; k >= 0; --k) {
                h += calib.allK.at<double>(0, k) * phiPow;
                phiPow *= phi;
            }
            height.at<double>(r, c) = h;
        }
    }
    return height;
}

// 功能：深度梯度滤波，输入：heightMap, mask, gradThresh，返回：过滤后掩膜
cv::Mat HeightCalculator::depthGradientFilter(
    const cv::Mat& heightMap, const cv::Mat& mask,
    double gradThresh)
{
    if (heightMap.empty() || mask.empty()) {
        throw tp::ImageLoadException("depthGradientFilter: empty input");
    }
    const int h = heightMap.rows;
    const int w = heightMap.cols;
    constexpr int MIN_CONNECTED = 500;

    // 计算4邻域最大深度差
    cv::Mat maxDiff = cv::Mat::zeros(h, w, CV_64F);
    for (int i = 1; i < h - 1; ++i) {
        for (int j = 1; j < w - 1; ++j) {
            double cur = heightMap.at<double>(i, j);
            double n4[4] = {
                heightMap.at<double>(i - 1, j),
                heightMap.at<double>(i + 1, j),
                heightMap.at<double>(i, j - 1),
                heightMap.at<double>(i, j + 1)};
            double mx = *std::max_element(n4, n4 + 4);
            double mn = *std::min_element(n4, n4 + 4);
            maxDiff.at<double>(i, j) =
                std::max(std::abs(cur - mx), std::abs(cur - mn));
        }
    }
    // 边界处理(简化: 2邻域或3邻域)
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            if (i > 0 && i < h - 1 && j > 0 && j < w - 1) continue;
            double cur = heightMap.at<double>(i, j);
            double mxD = 0.0;
            auto check = [&](int r2, int c2) {
                double d = std::abs(cur - heightMap.at<double>(r2, c2));
                if (d > mxD) mxD = d;
            };
            if (i > 0) check(i - 1, j);
            if (i < h - 1) check(i + 1, j);
            if (j > 0) check(i, j - 1);
            if (j < w - 1) check(i, j + 1);
            maxDiff.at<double>(i, j) = mxD;
        }
    }

    // 自适应阈值: 掩膜内非零梯度第50百分位 × 10
    cv::Mat gradMasked = maxDiff.mul(mask);
    std::vector<double> nonZeroGrads;
    nonZeroGrads.reserve(h * w / 4);
    for (int i = 0; i < h; ++i)
        for (int j = 0; j < w; ++j)
            if (gradMasked.at<double>(i, j) > 0)
                nonZeroGrads.push_back(gradMasked.at<double>(i, j));

    double adaptiveThresh = gradThresh;
    if (!nonZeroGrads.empty()) {
        std::sort(nonZeroGrads.begin(), nonZeroGrads.end());
        size_t idx50 = static_cast<size_t>(0.5 * nonZeroGrads.size());
        idx50 = std::min(idx50, nonZeroGrads.size() - 1);
        adaptiveThresh = 10.0 * nonZeroGrads[idx50];
    }

    // 梯度阈值化
    cv::Mat result = cv::Mat::zeros(h, w, CV_8U);
    for (int i = 0; i < h; ++i)
        for (int j = 0; j < w; ++j)
            if (maxDiff.at<double>(i, j) < adaptiveThresh)
                result.at<uchar>(i, j) = 1;
    // 与先验掩膜交
    cv::Mat mask8u;
    mask.convertTo(mask8u, CV_8U);
    cv::bitwise_and(result, mask8u, result);

    // 去除小连通域
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(
        result, labels, stats, centroids, 8, CV_32S);
    for (int lb = 1; lb < nLabels; ++lb) {
        if (stats.at<int>(lb, cv::CC_STAT_AREA) < MIN_CONNECTED) {
            cv::Mat lbMask = (labels == lb);
            result.setTo(0, lbMask);
        }
    }
    return result;
}

// 功能：多特征融合质量掩膜，输入：展开相位, 原始图像, 包裹相位, steps
cv::Mat HeightCalculator::phaseQualityMask(
    const cv::Mat& unwrappedPhase,
    const std::vector<cv::Mat>& imgs,
    const std::vector<cv::Mat>& wrappedPhases,
    int steps)
{
    if (unwrappedPhase.empty() || imgs.empty() || wrappedPhases.size() < 3) {
        throw tp::ImageLoadException("phaseQualityMask: invalid input");
    }
    const int h = unwrappedPhase.rows;
    const int w = unwrappedPhase.cols;
    constexpr double PHASE_GRAD_THRESH = 2.0 * M_PI;
    constexpr double DATAMOD_THRESH = 0.2;
    constexpr double ERROR_THRESH = 0.3;
    constexpr int MIN_CONNECTED = 100;

    PhaseDecoder decoder;

    // 误差能量掩膜 (3频合并)
    cv::Mat errorMask = cv::Mat::ones(h, w, CV_8U);
    for (int freq = 0; freq < 3; ++freq) {
        // 提取该频率的图像 (假设imgs按频率×步数排列)
        std::vector<cv::Mat> freqImgs(
            imgs.begin() + freq * steps,
            imgs.begin() + freq * steps + steps);
        cv::Mat errEnergy = decoder.calcPhaseErrorEnergy(
            freqImgs, wrappedPhases[freq], steps);
        cv::Mat eMask;
        cv::threshold(errEnergy, eMask, ERROR_THRESH, 1.0, cv::THRESH_BINARY_INV);
        eMask.convertTo(eMask, CV_8U);
        cv::bitwise_and(errorMask, eMask, errorMask);
    }

    // 调制度对比度掩膜
    std::vector<cv::Mat> freq0Imgs(imgs.begin(), imgs.begin() + steps);
    cv::Mat contrastMap = decoder.calcModulationContrast(freq0Imgs, steps);
    cv::Mat dataMask;
    cv::threshold(contrastMap, dataMask, DATAMOD_THRESH, 1.0, cv::THRESH_BINARY);
    dataMask.convertTo(dataMask, CV_8U);

    // 相位梯度掩膜 (4邻域)
    cv::Mat gradMask = cv::Mat::ones(h, w, CV_8U);
    for (int i = 1; i < h - 1; ++i) {
        for (int j = 1; j < w - 1; ++j) {
            double cur = unwrappedPhase.at<double>(i, j);
            double n4[4] = {
                unwrappedPhase.at<double>(i - 1, j),
                unwrappedPhase.at<double>(i + 1, j),
                unwrappedPhase.at<double>(i, j - 1),
                unwrappedPhase.at<double>(i, j + 1)};
            double mx = *std::max_element(n4, n4 + 4);
            double mn = *std::min_element(n4, n4 + 4);
            double maxD = std::max(std::abs(cur - mx), std::abs(cur - mn));
            if (maxD >= PHASE_GRAD_THRESH) {
                gradMask.at<uchar>(i, j) = 0;
            }
        }
    }

    // 三层掩膜取交集
    cv::Mat finalMask;
    cv::bitwise_and(errorMask, dataMask, finalMask);
    cv::bitwise_and(finalMask, gradMask, finalMask);

    // 去除小连通域
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(
        finalMask, labels, stats, centroids, 8, CV_32S);
    for (int lb = 1; lb < nLabels; ++lb) {
        if (stats.at<int>(lb, cv::CC_STAT_AREA) < MIN_CONNECTED) {
            finalMask.setTo(0, labels == lb);
        }
    }
    return finalMask;
}

// 功能：调制度/亮度掩膜生成，输入：imgs(3freq×N), steps
cv::Mat HeightCalculator::generateMask(
    const std::vector<cv::Mat>& imgs, int steps)
{
    if (imgs.empty()) {
        throw tp::ImageLoadException("generateMask: imgs is empty");
    }
    constexpr double MOD_RATIO_THRESH = 0.1;
    constexpr int NUM_FREQ = 3;

    const int rows = imgs[0].rows;
    const int cols = imgs[0].cols;
    cv::Mat mask = cv::Mat::ones(rows, cols, CV_8U);

    for (int freq = 0; freq < NUM_FREQ; ++freq) {
        cv::Mat meanPic = cv::Mat::zeros(rows, cols, CV_64F);
        cv::Mat sinP = cv::Mat::zeros(rows, cols, CV_64F);
        cv::Mat cosP = cv::Mat::zeros(rows, cols, CV_64F);
        for (int n = 0; n < steps; ++n) {
            int idx = freq * steps + n;
            if (idx >= static_cast<int>(imgs.size())) break;
            cv::Mat img64;
            imgs[idx].convertTo(img64, CV_64F);
            meanPic += img64;
            double angle = -2.0 * M_PI * n / steps;
            sinP += img64 * std::sin(angle);
            cosP += img64 * std::cos(angle);
        }
        meanPic /= steps;
        cv::Mat mod;
        cv::magnitude(sinP, cosP, mod);
        mod *= (2.0 / steps);

        // delta = B / A
        cv::Mat safeMean;
        cv::max(meanPic, 1e-10, safeMean);
        cv::Mat delta;
        cv::divide(mod, safeMean, delta);

        cv::Mat freqMask;
        cv::threshold(delta, freqMask, MOD_RATIO_THRESH, 1.0, cv::THRESH_BINARY);
        freqMask.convertTo(freqMask, CV_8U);
        cv::bitwise_and(mask, freqMask, mask);
    }
    return mask;
}

} // namespace tp
