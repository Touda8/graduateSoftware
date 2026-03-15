#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace tp {

class PointCloudIO {
public:
    using CloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
    using ColorCloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

    static void savePLY(CloudPtr cloud, const std::string& path);
    static void savePLY(ColorCloudPtr cloud, const std::string& path);
    static void savePCD(CloudPtr cloud, const std::string& path);
    static void savePCD(ColorCloudPtr cloud, const std::string& path);
    static void saveCSV(CloudPtr cloud, const std::string& path);
    static void saveCSV(ColorCloudPtr cloud, const std::string& path);

    // 加载点云
    static CloudPtr loadPLY(const std::string& path);
    static CloudPtr loadPCD(const std::string& path);
    static CloudPtr load(const std::string& path);
};

} // namespace tp