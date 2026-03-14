#include "pointcloud/PointCloudFilter.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

namespace tp {

PointCloudFilter::CloudPtr PointCloudFilter::SOR(
    CloudPtr cloud, int meanK, double stddevMul)
{
    if (!cloud || cloud->empty()) {
        Logger::instance().warn("SOR: input cloud is empty, returning as-is");
        return cloud;
    }
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMul);
    sor.filter(*filtered);

    Logger::instance().info("SOR: " + std::to_string(cloud->size()) +
        " -> " + std::to_string(filtered->size()) + " points");

    if (filtered->size() < 10) {
        throw FilterException("SOR: output has fewer than 10 points ("
            + std::to_string(filtered->size()) + ")");
    }
    return filtered;
}

// voxelGrid, passThrough 将在此后插入

PointCloudFilter::CloudPtr PointCloudFilter::voxelGrid(
    CloudPtr cloud, float leafSize)
{
    if (!cloud || cloud->empty()) {
        Logger::instance().warn("voxelGrid: input cloud is empty");
        return cloud;
    }
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*filtered);
    Logger::instance().info("voxelGrid: " + std::to_string(cloud->size()) +
        " -> " + std::to_string(filtered->size()));
    return filtered;
}

PointCloudFilter::CloudPtr PointCloudFilter::passThrough(
    CloudPtr cloud, const std::string& field, float min, float max)
{
    if (!cloud || cloud->empty()) {
        Logger::instance().warn("passThrough: input cloud is empty");
        return cloud;
    }
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PassThrough<pcl::PointXYZ> pt;
    pt.setInputCloud(cloud);
    pt.setFilterFieldName(field);
    pt.setFilterLimits(min, max);
    pt.filter(*filtered);
    Logger::instance().info("passThrough(" + field + "): " +
        std::to_string(cloud->size()) + " -> " +
        std::to_string(filtered->size()));
    return filtered;
}

} // namespace tp