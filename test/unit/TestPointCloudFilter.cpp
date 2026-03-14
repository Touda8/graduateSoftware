// TestPointCloudFilter.cpp - PointCloudFilter 单元测试
#include "TestFramework.h"
#include "pointcloud/PointCloudFilter.h"
#include "common/Exceptions.h"
#include <random>

TEST(PointCloudFilter, SOR_RemovesOutliers) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::mt19937 rng(42);
    std::normal_distribution<float> dist(0.0f, 1.0f);
    for (int i = 0; i < 500; ++i) {
        cloud->push_back({dist(rng), dist(rng), dist(rng)});
    }
    for (int i = 0; i < 50; ++i) {
        cloud->push_back({100.0f + dist(rng),
                          100.0f + dist(rng),
                          100.0f + dist(rng)});
    }
    ASSERT_EQ(static_cast<int>(cloud->size()), 550);

    tp::PointCloudFilter filter;
    auto filtered = filter.SOR(cloud, 30, 1.0);

    ASSERT_LT(static_cast<int>(filtered->size()), 550);
    ASSERT_GT(static_cast<int>(filtered->size()), 400);
}

TEST(PointCloudFilter, SOR_EmptyCloudSafe) {
    auto empty = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    tp::PointCloudFilter filter;
    ASSERT_NO_THROW(filter.SOR(empty));
}

