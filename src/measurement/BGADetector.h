#pragma once

#include "common/interfaces.h"
#include <cmath>

namespace tp {

// allDealTwoplane 算法配置参数
struct BGAConfig {
    double circularityThresh = 0.85;
    int areaMin = 300;
    int areaMax = 4000;
    int edgeDistThresh = 1;
    int minBallPoints = 20;
    double zFlip = 160.0;
    int morphCloseRadius = 3;
};

// 单个焊球测量结果
struct BallResult {
    int order = 0;         // 排序编号
    int row = 0;           // 所在行号
    double xc = 0, yc = 0; // 圆心(像素)
    double radius = 0;     // 等效半径
    double x3d = 0, y3d = 0, z3d = 0; // 3D质心(mm)
    double height = 0;     // 到基板平面距离(mm)
    double nx = 0, ny = 0, nz = 0;    // 焊球PCA法向量
    bool success = false;
};

// detectBallMask 返回的掩膜集合
struct DetectionMasks {
    cv::Mat ballMask;      // 最终焊球掩膜
    cv::Mat chipMask;      // 芯片区域掩膜 (255=chip)
};

class BGADetector : public IMeasure {
public:
    BGADetector() = default;
    ~BGADetector() noexcept override = default;

    // IMeasure interface (兼容旧接口)
    std::vector<cv::Vec3f> detect2DBalls(
        const cv::Mat& brightImage, const cv::Mat& mask) override;
    std::vector<Eigen::Vector3d> localize3DBalls(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const std::vector<cv::Vec3f>& balls2D,
        const CalibData& calib) override;
    double calcCoplanarity(
        const std::vector<Eigen::Vector3d>& points) override;

    // allDealTwoplane 全流程（新算法入口）
    // 输入: 2D图像, 标定X/Y矩阵, 高度图Z, 配置
    // 输出: BallResult 列表，可选地生成可视化图
    std::vector<BallResult> runAllDealTwoplane(
        const cv::Mat& bgaImage,
        const cv::Mat& X, const cv::Mat& Y, const cv::Mat& Z,
        const BGAConfig& config = {});

    // 获取上一次测量的基板法向量和d参数
    Eigen::Vector3d getSubstrateNormal() const { return substrateNormal_; }
    double getSubstrateD() const { return substrateD_; }

private:
    // 生成焊球掩膜（芯片定位+Otsu+形态学+筛选），同时返回芯片掩膜
    DetectionMasks detectBallMask(const cv::Mat& gray, const BGAConfig& cfg);

    // PCA平面拟合，返回法向量。centroid和d通过引用返回
    Eigen::Vector3d fitPCAPlane(const std::vector<Eigen::Vector3d>& pts,
                                Eigen::Vector3d& centroid, double& d);

    // 存储基板平面参数
    Eigen::Vector3d substrateNormal_{0, 0, -1};
    double substrateD_ = 0;
};

} // namespace tp
