#pragma once

#include "common/interfaces.h"
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <array>

namespace tp {

// QFP引脚区域信息
struct PinRegion {
    int pinId = 0;
    std::string side;          // "top"/"bottom"/"left"/"right"
    cv::Rect bbox;
    cv::Point2d centroid;
    double area = 0;
    cv::Mat mask;              // 单引脚掩膜
};

// QFP芯片几何信息
struct ChipInfo {
    std::array<cv::Point2d, 4> corners; // 左上/右上/右下/左下
    cv::Vec3d lineTop, lineBot;         // ax+by+c=0
    cv::Vec3d lineLeft, lineRight;
};

// 引脚底部点云
struct PinBottomCloud {
    int pinId = 0;
    std::string side;
    std::vector<Eigen::Vector3d> fullCloud;
    std::vector<Eigen::Vector3d> bottomCloud;
    std::vector<Eigen::Vector3d> refinedCloud;
    Eigen::Vector4d localPlane;   // (A,B,C,D)
    double deviation = 0;         // 共面度偏差
    double warpAngle = 0;         // 翘曲角(度)
};

// 共面度结果
struct QFPCoplanarityResult {
    double coplanarityValue = 0;  // JEDEC共面度(mm)
    double maxDeviation = 0;
    Eigen::Vector4d seatingPlane; // 座落平面
    std::vector<int> seatingPinIds;
    std::vector<PinBottomCloud> pins;
};

class BGAMeasurePipeline {
public:
    BGAMeasurePipeline() = default;
    ~BGAMeasurePipeline() noexcept = default;

    // === 分割阶段 ===
    // Step1: 芯片区域定位(分层Otsu + IRLS边界拟合)
    ChipInfo localizeChip(const cv::Mat& gray, cv::Mat& maskOut);
    // Step2: 封装本体边界精化
    cv::Rect refineBodyBoundary(const cv::Mat& gray,
                                const cv::Mat& chipMask);
    // Step3: 引脚搜索区域确定
    std::vector<cv::Rect> defineSearchRegions(
        const ChipInfo& info, const cv::Size& imgSize);
    // Step4: 几何过滤 + Step5: 引脚分割
    std::vector<PinRegion> segmentPins(
        const cv::Mat& gray, const std::vector<cv::Rect>& regions,
        const cv::Rect& bodyRect);

    // === 计算阶段 ===
    // Step1: 封装本体曲面拟合
    std::array<double, 6> fitBodySurface(
        const cv::Mat& X, const cv::Mat& Y, const cv::Mat& Z,
        const cv::Mat& bodyMask, Eigen::Vector3d& normalOut);
    // Step2: 引脚底部提取
    std::vector<PinBottomCloud> extractPinBottoms(
        const cv::Mat& X, const cv::Mat& Y, const cv::Mat& Z,
        const std::vector<PinRegion>& pins);
    // Step3: 点云精化(法向量+连通性)
    void refineBottomClouds(
        std::vector<PinBottomCloud>& pins,
        const Eigen::Vector3d& bodyNormal);
    // Step4: JEDEC共面度计算
    QFPCoplanarityResult calcQFPCoplanarity(
        std::vector<PinBottomCloud>& pins);

private:
    // IRLS直线拟合: 返回ax+by+c=0
    cv::Vec3d irlsLineFit(const std::vector<cv::Point2d>& pts,
                          int maxIter, double sigma);
    // 两直线交点
    static cv::Point2d lineIntersection(
        const cv::Vec3d& l1, const cv::Vec3d& l2);
};

} // namespace tp
