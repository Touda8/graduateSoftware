#pragma once

#include "common/interfaces.h"
#include <cmath>

namespace tp {

class HeightCalculator : public IHeightCalc {
public:
    HeightCalculator() = default;
    ~HeightCalculator() noexcept override = default;

    // IHeightCalc interface
    cv::Mat calcPlaneHeight(
        const cv::Mat& absPhase, const CalibData& calib) override;
    cv::Mat depthGradientFilter(
        const cv::Mat& heightMap, const cv::Mat& mask,
        double gradThresh) override;
    cv::Mat phaseQualityMask(
        const cv::Mat& unwrappedPhase,
        const std::vector<cv::Mat>& imgs,
        const std::vector<cv::Mat>& wrappedPhases,
        int steps) override;
    cv::Mat generateMask(
        const std::vector<cv::Mat>& imgs, int steps) override;

private:
    // PCA平面拟合, 返回(A,B,C,D)使 Ax+By+Cz+D=0
    Eigen::Vector4d fitPlanePCA(const cv::Mat& points);
};

} // namespace tp
