#pragma once

#include "common/interfaces.h"
#include <cmath>

namespace tp {

class BGADetector : public IMeasure {
public:
    BGADetector() = default;
    ~BGADetector() noexcept override = default;

    // IMeasure interface
    std::vector<cv::Vec3f> detect2DBalls(
        const cv::Mat& brightImage, const cv::Mat& mask) override;
    std::vector<Eigen::Vector3d> localize3DBalls(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const std::vector<cv::Vec3f>& balls2D,
        const CalibData& calib) override;
    double calcCoplanarity(
        const std::vector<Eigen::Vector3d>& points) override;

private:
    // 迭代圆拟合：返回(cx, cy, r)
    cv::Vec3d fitCircleIterative(
        const std::vector<cv::Point2d>& pts, int maxIter, double beta);
    // 二阶曲面拟合: Z = ax²+by²+cxy+dx+ey+f
    std::array<double, 6> fitQuadSurface(
        const std::vector<Eigen::Vector3d>& pts);
};

} // namespace tp
