#pragma once

// interfaces.h - 公共接口定义
// 包含所有模块的纯虚基类和公共数据结构

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <memory>
#include <filesystem>

namespace tp {

// 标定数据（来自 .raw/.mat 标定文件）
struct CalibData {
    cv::Mat allK;       // (1 x m) 相位-高度多项式系数
    cv::Mat allK_i;     // 逆映射系数
    cv::Mat X;          // 世界坐标 X 映射矩阵
    cv::Mat Y;          // 世界坐标 Y 映射矩阵
    bool isValid = false;
};

// 重建参数（来自 UI 控件）
struct ReconParams {
    int freq1 = 64, freq2 = 56, freq3 = 63;
    int shift1 = 8, shift2 = 8, shift3 = 8;
    double modThresh      = 0.7;
    double depthMin       = 50.0;     // mm
    double depthMax       = 210.0;    // mm
    double phaseThresh    = 0.04;
    double epiThresh      = 1.5;      // px
    double centroidThresh = 0.05;
    double tMax           = 10.0;
    double numStable      = 0.001;
    double fsnrHigh       = 0.3;
    double fsnrLow        = 0.15;
    double modHigh        = 30.0;
    double modLow         = 10.0;
    bool   orderedCloud   = true;
    bool   depthRangeEnabled = true;
    int    repeatCount    = 1;
    std::string decodeType = "multi_freq";
};

// 重建结果
struct ReconResult {
    cv::Mat absPhase1;     // Pro1 绝对相位图
    cv::Mat absPhase2;     // Pro2 绝对相位图
    cv::Mat heightMap;     // 融合高度图
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    bool success = false;
    std::string errorMsg;
};

// BGA 测量结果
struct BGAMeasureResult {
    std::vector<Eigen::Vector3d> ballPositions;
    double coplanarity = 0.0;   // 共面度 (mm)
    double maxDev      = 0.0;   // 最大偏差
    bool   success     = false;
    std::string errorMsg;
};

// 相位解码器接口
class IDecoder {
public:
    virtual ~IDecoder() noexcept = default;

    // N步相移主值相位计算，返回包裹相位 [-π, π]
    virtual cv::Mat calcPhaseMatrix(
        const std::vector<cv::Mat>& imgs, int steps) = 0;

    // 三频绝对相位展开
    virtual cv::Mat multiFreqUnwrap(
        const cv::Mat& phi1, const cv::Mat& phi2, const cv::Mat& phi3,
        int f1, int f2, int f3) = 0;

    // 条纹调制度计算
    virtual cv::Mat calcFringeModulation(
        const std::vector<cv::Mat>& imgs, int steps) = 0;

    // 调制度对比度计算
    virtual cv::Mat calcModulationContrast(
        const std::vector<cv::Mat>& imgs, int steps) = 0;

    // 相位误差能量计算
    virtual cv::Mat calcPhaseErrorEnergy(
        const std::vector<cv::Mat>& imgs,
        const cv::Mat& wrappedPhase, int steps) = 0;

    // 时间相位展开（粗→精）
    virtual cv::Mat unwrapPhase(
        const cv::Mat& keyPhase, int multiple,
        const cv::Mat& wrappedPhase) = 0;
};

// 高度计算器接口
class IHeightCalc {
public:
    virtual ~IHeightCalc() noexcept = default;

    // 由绝对相位和标定系数计算高度图
    virtual cv::Mat calcPlaneHeight(
        const cv::Mat& absPhase, const CalibData& calib) = 0;

    // 深度梯度滤波（去除深度突变点）
    virtual cv::Mat depthGradientFilter(
        const cv::Mat& heightMap, const cv::Mat& mask,
        double gradThresh) = 0;

    // 多特征融合质量掩膜
    virtual cv::Mat phaseQualityMask(
        const cv::Mat& unwrappedPhase,
        const std::vector<cv::Mat>& imgs,
        const std::vector<cv::Mat>& wrappedPhases,
        int steps) = 0;

    // 调制度/亮度掩膜生成
    virtual cv::Mat generateMask(
        const std::vector<cv::Mat>& imgs, int steps) = 0;
};

// 点云滤波器接口
class IFilter {
public:
    virtual ~IFilter() noexcept = default;

    // 统计离群值滤波
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr SOR(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input,
        int k, double stdMul) = 0;

    // 半径离群值滤波
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr ROR(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input,
        double radius, int minNeighbors) = 0;

    // 直通滤波
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input,
        const std::string& axis, float min, float max) = 0;

    // 体素降采样
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGrid(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input,
        float leafSize) = 0;
};

// BGA/QFP 测量接口
class IMeasure {
public:
    virtual ~IMeasure() noexcept = default;

    // BGA 二维焊球检测
    virtual std::vector<cv::Vec3f> detect2DBalls(
        const cv::Mat& brightImage, const cv::Mat& mask) = 0;

    // BGA 三维焊球顶点定位
    virtual std::vector<Eigen::Vector3d> localize3DBalls(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const std::vector<cv::Vec3f>& balls2D,
        const CalibData& calib) = 0;

    // 共面度计算（BGA/QFP 通用）
    virtual double calcCoplanarity(
        const std::vector<Eigen::Vector3d>& points) = 0;
};

} // namespace tp
