// TestReconstructionPipeline.cpp - 集成测试
#include "TestFramework.h"
#include "reconstruction/CalibLoader.h"
#include "reconstruction/ReconstructionPipeline.h"
#include "reconstruction/PhaseDecoder.h"
#include "pointcloud/PointCloudIO.h"
#include "common/Exceptions.h"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>

namespace {
// Windows ANSI 代码页无法处理 UTF-8 中文路径，用 filesystem::path + imdecode 绕过
cv::Mat imreadSafe(const std::string& utf8Path, int flags) {
    std::ifstream ifs(std::filesystem::u8path(utf8Path), std::ios::binary);
    if (!ifs) return {};
    std::vector<uchar> buf(std::istreambuf_iterator<char>(ifs), {});
    return cv::imdecode(buf, flags);
}
} // namespace

TEST(ReconPipeline, MissingCalibFile) {
    tp::CalibLoader loader;
    ASSERT_THROW(loader.load("nonexistent_dir_XXXXXX"),
                 tp::CalibException);
}

TEST(ReconPipeline, CalibLoader_ValidDir) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/calibrationResult/raw")) {
        std::cout << "  [SKIP] raw/ not found\n";
        ASSERT_TRUE(true);
        return;
    }
    tp::CalibLoader loader;
    auto calib = loader.load("data/calibrationResult");
    ASSERT_TRUE(calib.isValid);
    ASSERT_EQ(calib.X.rows, 2048);
    ASSERT_EQ(calib.X.cols, 2448);
    ASSERT_EQ(calib.allK.rows, 5013504);
    ASSERT_EQ(calib.allK.cols, 3);
}

TEST(FullPipeline, LoadDualCalib) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/calibrationResult/raw")) {
        std::cout << "  [SKIP] raw/ not found\n";
        ASSERT_TRUE(true); return;
    }
    tp::CalibLoader loader;
    auto dc = loader.loadDual("data/calibrationResult/raw");
    ASSERT_TRUE(dc.isValid);
    ASSERT_EQ(dc.imgRows, 2048);
    ASSERT_EQ(dc.imgCols, 2448);
    ASSERT_EQ(dc.allK_i1.rows, 5013504);
    ASSERT_EQ(dc.allK_i1.cols, 3);
    ASSERT_EQ(dc.allK_i2.rows, 5013504);
    ASSERT_EQ(dc.allK_i2.cols, 3);
    ASSERT_GT(dc.X.rows, 0);
}

TEST(FullPipeline, LoadImages) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/pic/BGA静态/1")) {
        std::cout << "  [SKIP] image dir not found\n";
        ASSERT_TRUE(true); return;
    }
    std::vector<cv::Mat> imgs;
    for (int i = 1; i <= 24; ++i) {
        std::string p = "data/pic/BGA静态/1/" + std::to_string(i) + ".bmp";
        cv::Mat img = imreadSafe(p, cv::IMREAD_GRAYSCALE);
        ASSERT_FALSE(img.empty());
        imgs.push_back(img);
    }
    ASSERT_EQ(imgs.size(), 24u);
    ASSERT_EQ(imgs[0].rows, 2048);
    ASSERT_EQ(imgs[0].cols, 2448);
}

TEST(FullPipeline, PhaseDecoding) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/pic/BGA静态/1")) {
        std::cout << "  [SKIP] image dir not found\n";
        ASSERT_TRUE(true); return;
    }
    std::vector<cv::Mat> freq1;
    for (int i = 1; i <= 8; ++i) {
        std::string p = "data/pic/BGA静态/1/" + std::to_string(i) + ".bmp";
        cv::Mat img = imreadSafe(p, cv::IMREAD_GRAYSCALE);
        cv::Mat d; img.convertTo(d, CV_64F); freq1.push_back(d);
    }
    tp::PhaseDecoder decoder;
    cv::Mat phi1 = decoder.calcPhaseMatrix(freq1, 8);
    ASSERT_EQ(phi1.rows, freq1[0].rows);
    ASSERT_EQ(phi1.cols, freq1[0].cols);
    double mn, mx;
    cv::minMaxLoc(phi1, &mn, &mx);
    ASSERT_FALSE(std::isnan(mn));
    ASSERT_FALSE(std::isinf(mn));
}

TEST(FullPipeline, EndToEnd) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/calibrationResult/raw") ||
        !fs::is_directory("data/pic/BGA静态/1")) {
        std::cout << "  [SKIP] data not found\n";
        ASSERT_TRUE(true); return;
    }
    tp::CalibLoader loader;
    auto dc = loader.loadDual("data/calibrationResult/raw");
    ASSERT_TRUE(dc.isValid);

    auto loadImgs = [](const std::string& folder) {
        std::vector<cv::Mat> imgs;
        for (int i = 1; i <= 24; ++i) {
            cv::Mat img = imreadSafe(
                folder + "/" + std::to_string(i) + ".bmp",
                cv::IMREAD_GRAYSCALE);
            cv::Mat d; img.convertTo(d, CV_64F, 1.0 / 255.0);
            imgs.push_back(d);
        }
        return imgs;
    };
    auto imgs1 = loadImgs("data/pic/BGA静态/1");
    auto imgs2 = loadImgs("data/pic/BGA静态/2");
    ASSERT_EQ(imgs1.size(), 24u);
    ASSERT_EQ(imgs2.size(), 24u);

    cv::Mat colorImg = imreadSafe(
        "data/pic/BGA静态/1/49.bmp", cv::IMREAD_GRAYSCALE);
    ASSERT_FALSE(colorImg.empty());

    tp::ReconstructionPipeline pipeline;
    tp::ReconParams params;
    auto result = pipeline.runDual(
        imgs1, imgs2, colorImg, dc, params);
    ASSERT_TRUE(result.success);
    ASSERT_GT(result.cloud->size(), 1000u);
    std::cout << "  EndToEnd: " << result.cloud->size()
              << " points\n";

    for (size_t i = 0;
         i < std::min<size_t>(100, result.cloud->size()); ++i) {
        ASSERT_FALSE(std::isnan(result.cloud->points[i].x));
        ASSERT_FALSE(std::isnan(result.cloud->points[i].z));
    }
}

TEST(FullPipeline, PointCloudIORoundtrip) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->push_back({1.f, 2.f, 3.f});
    cloud->push_back({4.f, 5.f, 6.f});

    std::string path = "test_roundtrip_tmp.ply";
    tp::PointCloudIO::savePLY(cloud, path);

    auto loaded = tp::PointCloudIO::loadPLY(path);
    ASSERT_EQ(loaded->size(), 2u);
    ASSERT_NEAR((*loaded)[0].x, 1.f, 1e-4f);
    ASSERT_NEAR((*loaded)[1].z, 6.f, 1e-4f);

    std::filesystem::remove(path);
}

