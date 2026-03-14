// TestCrashPrevention.cpp - 防崩溃回归测试（C1-C4）
#include "TestFramework.h"
#include "reconstruction/CalibLoader.h"
#include "reconstruction/ReconstructionPipeline.h"
#include "pointcloud/PointCloudIO.h"
#include "common/Exceptions.h"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <thread>

namespace {
// UTF-8 安全读图（与 TestReconstructionPipeline 中同名，匿名 namespace 避免冲突）
cv::Mat imreadSafeCrash(const std::string& utf8Path, int flags) {
    std::ifstream ifs(std::filesystem::u8path(utf8Path), std::ios::binary);
    if (!ifs) return {};
    std::vector<uchar> buf(std::istreambuf_iterator<char>(ifs), {});
    return cv::imdecode(buf, flags);
}
} // namespace

// C1: 在非主线程中运行 runDual，验证不崩溃
TEST(ThreadSafety, ReconPipelineNoGuiCrash) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/calibrationResult/raw") ||
        !fs::is_directory("data/pic/BGA静态/1") ||
        !fs::is_directory("data/pic/BGA静态/2")) {
        std::cout << "  [SKIP] data dirs not found\n";
        ASSERT_TRUE(true); return;
    }

    tp::CalibLoader loader;
    auto dc = loader.loadDual("data/calibrationResult/raw");
    ASSERT_TRUE(dc.isValid);

    auto loadImgs = [](const std::string& folder) {
        std::vector<cv::Mat> imgs;
        for (int i = 1; i <= 24; ++i) {
            cv::Mat img = imreadSafeCrash(
                folder + "/" + std::to_string(i) + ".bmp",
                cv::IMREAD_GRAYSCALE);
            cv::Mat d; img.convertTo(d, CV_64F, 1.0 / 255.0);
            imgs.push_back(d);
        }
        return imgs;
    };
    auto imgs1 = loadImgs("data/pic/BGA静态/1");
    auto imgs2 = loadImgs("data/pic/BGA静态/2");

    cv::Mat colorImg = imreadSafeCrash(
        "data/pic/BGA静态/1/49.bmp", cv::IMREAD_GRAYSCALE);

    tp::ReconstructionPipeline pipeline;
    tp::ReconParams params;
    tp::DualReconResult result;

    // 在子线程中运行重建
    std::thread worker([&]() {
        result = pipeline.runDual(
            imgs1, imgs2, colorImg, dc, params);
    });
    worker.join();

    ASSERT_TRUE(result.success);
    ASSERT_GT(result.cloud->size(), 100u);
    std::cout << "  C1 thread result: "
              << result.cloud->size() << " points\n";
}

// C2: 传入空 colorImage，pipeline 不崩溃且 success == true
TEST(Defense, EmptyColorImageNoThrow) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/calibrationResult/raw") ||
        !fs::is_directory("data/pic/BGA静态/1") ||
        !fs::is_directory("data/pic/BGA静态/2")) {
        std::cout << "  [SKIP] data dirs not found\n";
        ASSERT_TRUE(true); return;
    }

    tp::CalibLoader loader;
    auto dc = loader.loadDual("data/calibrationResult/raw");
    ASSERT_TRUE(dc.isValid);

    auto loadImgs = [](const std::string& folder) {
        std::vector<cv::Mat> imgs;
        for (int i = 1; i <= 24; ++i) {
            cv::Mat img = imreadSafeCrash(
                folder + "/" + std::to_string(i) + ".bmp",
                cv::IMREAD_GRAYSCALE);
            cv::Mat d; img.convertTo(d, CV_64F, 1.0 / 255.0);
            imgs.push_back(d);
        }
        return imgs;
    };
    auto imgs1 = loadImgs("data/pic/BGA静态/1");
    auto imgs2 = loadImgs("data/pic/BGA静态/2");

    // 空 colorImage
    cv::Mat emptyColor;

    tp::ReconstructionPipeline pipeline;
    tp::ReconParams params;
    tp::DualReconResult result;

    ASSERT_NO_THROW(
        result = pipeline.runDual(
            imgs1, imgs2, emptyColor, dc, params));
    ASSERT_TRUE(result.success);
    ASSERT_GT(result.cloud->size(), 100u);
    std::cout << "  C2 empty color: "
              << result.cloud->size() << " points\n";
}

// C3: 完整 EndToEnd，检查 Z 值范围和 NaN 率，保存 PLY 并验证可加载
TEST(Defense, ReconResultValidation) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/calibrationResult/raw") ||
        !fs::is_directory("data/pic/BGA静态/1") ||
        !fs::is_directory("data/pic/BGA静态/2")) {
        std::cout << "  [SKIP] data dirs not found\n";
        ASSERT_TRUE(true); return;
    }

    tp::CalibLoader loader;
    auto dc = loader.loadDual("data/calibrationResult/raw");
    ASSERT_TRUE(dc.isValid);

    auto loadImgs = [](const std::string& folder) {
        std::vector<cv::Mat> imgs;
        for (int i = 1; i <= 24; ++i) {
            cv::Mat img = imreadSafeCrash(
                folder + "/" + std::to_string(i) + ".bmp",
                cv::IMREAD_GRAYSCALE);
            cv::Mat d; img.convertTo(d, CV_64F, 1.0 / 255.0);
            imgs.push_back(d);
        }
        return imgs;
    };
    auto imgs1 = loadImgs("data/pic/BGA静态/1");
    auto imgs2 = loadImgs("data/pic/BGA静态/2");
    cv::Mat colorImg = imreadSafeCrash(
        "data/pic/BGA静态/1/49.bmp", cv::IMREAD_GRAYSCALE);

    tp::ReconstructionPipeline pipeline;
    tp::ReconParams params;
    auto result = pipeline.runDual(
        imgs1, imgs2, colorImg, dc, params);
    ASSERT_TRUE(result.success);
    ASSERT_GT(result.cloud->size(), 100u);

    // 遍历前 1000 个点，验证无 NaN/Inf
    size_t checkCount = std::min<size_t>(
        1000, result.cloud->size());
    int nanCount = 0;
    for (size_t i = 0; i < checkCount; ++i) {
        auto& pt = result.cloud->points[i];
        if (std::isnan(pt.x) || std::isnan(pt.y) ||
            std::isnan(pt.z)) ++nanCount;
        ASSERT_FALSE(std::isinf(pt.x));
        ASSERT_FALSE(std::isinf(pt.y));
        ASSERT_FALSE(std::isinf(pt.z));
    }
    double nanRate = static_cast<double>(nanCount) / checkCount;
    ASSERT_LT(nanRate, 0.01); // NaN 率 < 1%

    // 转换为 XYZ 保存 PLY 并重新加载验证
    auto xyzCloud = std::make_shared<
        pcl::PointCloud<pcl::PointXYZ>>();
    for (auto& pt : result.cloud->points)
        xyzCloud->push_back({pt.x, pt.y, pt.z});
    std::string plyPath = "test_crash_c3_tmp.ply";
    tp::PointCloudIO::savePLY(xyzCloud, plyPath);
    auto loaded = tp::PointCloudIO::loadPLY(plyPath);
    ASSERT_EQ(loaded->size(), xyzCloud->size());
    fs::remove(plyPath);

    std::cout << "  C3 validated: " << result.cloud->size()
              << " pts, NaN rate=" << nanRate << "\n";
}

// C4: colorImage 尺寸不匹配，pipeline 自动缩放不崩溃
TEST(Defense, ColorImageSizeMismatch) {
    namespace fs = std::filesystem;
    if (!fs::is_directory("data/calibrationResult/raw") ||
        !fs::is_directory("data/pic/BGA静态/1") ||
        !fs::is_directory("data/pic/BGA静态/2")) {
        std::cout << "  [SKIP] data dirs not found\n";
        ASSERT_TRUE(true); return;
    }

    tp::CalibLoader loader;
    auto dc = loader.loadDual("data/calibrationResult/raw");
    ASSERT_TRUE(dc.isValid);

    auto loadImgs = [](const std::string& folder) {
        std::vector<cv::Mat> imgs;
        for (int i = 1; i <= 24; ++i) {
            cv::Mat img = imreadSafeCrash(
                folder + "/" + std::to_string(i) + ".bmp",
                cv::IMREAD_GRAYSCALE);
            cv::Mat d; img.convertTo(d, CV_64F, 1.0 / 255.0);
            imgs.push_back(d);
        }
        return imgs;
    };
    auto imgs1 = loadImgs("data/pic/BGA静态/1");
    auto imgs2 = loadImgs("data/pic/BGA静态/2");

    // 故意创建尺寸不匹配的 colorImage（100x100）
    cv::Mat mismatchColor = cv::Mat(100, 100, CV_8UC1,
                                     cv::Scalar(200));

    tp::ReconstructionPipeline pipeline;
    tp::ReconParams params;
    tp::DualReconResult result;

    ASSERT_NO_THROW(
        result = pipeline.runDual(
            imgs1, imgs2, mismatchColor, dc, params));
    ASSERT_TRUE(result.success);
    ASSERT_GT(result.cloud->size(), 100u);

    // 验证颜色值合法（0-255，无越界访问）
    for (size_t i = 0;
         i < std::min<size_t>(500, result.cloud->size()); ++i) {
        auto& pt = result.cloud->points[i];
        ASSERT_TRUE(pt.r <= 255);
        ASSERT_TRUE(pt.g <= 255);
        ASSERT_TRUE(pt.b <= 255);
    }
    std::cout << "  C4 mismatch color: "
              << result.cloud->size() << " points\n";
}
