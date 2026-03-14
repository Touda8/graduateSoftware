#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace tp {

class PointCloudFilter {
public:
    using CloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

    // 统计离群值滤波
    CloudPtr SOR(CloudPtr cloud, int meanK = 50, double stddevMul = 1.0);

    // 体素网格下采样
    CloudPtr voxelGrid(CloudPtr cloud, float leafSize = 0.5f);

    // 直通滤波
    CloudPtr passThrough(CloudPtr cloud, const std::string& field,
                         float min, float max);
};

} // namespace tp