#pragma once

#include "reconstruction/PhaseDecoder.h"
#include "reconstruction/HeightCalculator.h"
#include "reconstruction/CalibLoader.h"
#include "common/interfaces.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <opencv2/core.hpp>

namespace tp {

// 双投影仪重建结果
struct DualReconResult {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cv::Mat absPhase1;   // 投影仪1绝对相位图
    cv::Mat absPhase2;   // 投影仪2绝对相位图
    cv::Mat heightMap;   // 融合高度图
    bool success = false;
    std::string errorMsg;
};

class ReconstructionPipeline {
public:
    using CloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
    using ColorCloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

    void setCalibData(const CalibData& calib);
    void setParams(const ReconParams& params);
    CloudPtr run(const std::vector<cv::Mat>& images);

    // 双投影仪重建
    DualReconResult runDual(
        const std::vector<cv::Mat>& images1,
        const std::vector<cv::Mat>& images2,
        const cv::Mat& colorImage,
        const DualCalibData& calib,
        const ReconParams& params);

private:
    CalibData calibData_;
    ReconParams params_;
    PhaseDecoder decoder_;
    HeightCalculator heightCalc_;
};

} // namespace tp