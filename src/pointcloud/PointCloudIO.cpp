#include "pointcloud/PointCloudIO.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <filesystem>
#include <vector>

namespace tp {

namespace {
// Windows 上 PCL 内部使用 fopen() 处理 ANSI 路径，无法加载含中文的 UTF-8 路径。
// 检测是否含有非 ASCII 字符；若有则复制到临时文件后加载。
bool hasNonAscii(const std::string& s) {
    for (unsigned char c : s) if (c > 127) return true;
    return false;
}

// 将文件复制到 %TEMP% 下的纯 ASCII 临时路径，返回临时路径
std::string copyToTempIfNeeded(const std::string& utf8Path,
                               const std::string& ext) {
    if (!hasNonAscii(utf8Path)) return utf8Path;
    auto srcPath = std::filesystem::u8path(utf8Path);
    if (!std::filesystem::exists(srcPath))
        throw IOException("file not found (u8): " + utf8Path);
    auto tmpDir = std::filesystem::temp_directory_path() / "tp_pcl_tmp";
    std::filesystem::create_directories(tmpDir);
    auto tmpFile = tmpDir / ("cloud_tmp" + ext);
    std::filesystem::copy_file(srcPath, tmpFile,
        std::filesystem::copy_options::overwrite_existing);
    return tmpFile.string();
}

void removeTempIfUsed(const std::string& original, const std::string& actual) {
    if (original != actual) {
        std::error_code ec;
        std::filesystem::remove(std::filesystem::path(actual), ec);
    }
}
} // namespace

void PointCloudIO::savePLY(CloudPtr cloud, const std::string& path) {
    if (!cloud || cloud->empty()) {
        throw IOException("savePLY: cloud is null or empty");
    }
    auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            throw IOException("savePLY: cannot create dir "
                + parent.string() + ": " + ec.message());
        }
    }
    if (pcl::io::savePLYFileBinary(path, *cloud) != 0) {
        throw IOException("savePLY: failed to write " + path);
    }
    Logger::instance().info("Saved PLY: " + path);
}

// savePCD, saveCSV 将在此后插入

void PointCloudIO::savePCD(CloudPtr cloud, const std::string& path) {
    if (!cloud || cloud->empty()) {
        throw IOException("savePCD: cloud is null or empty");
    }
    auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            throw IOException("savePCD: cannot create dir "
                + parent.string() + ": " + ec.message());
        }
    }
    if (pcl::io::savePCDFileBinary(path, *cloud) != 0) {
        throw IOException("savePCD: failed to write " + path);
    }
    Logger::instance().info("Saved PCD: " + path);
}

void PointCloudIO::saveCSV(CloudPtr cloud, const std::string& path) {
    if (!cloud || cloud->empty()) {
        throw IOException("saveCSV: cloud is null or empty");
    }
    auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            throw IOException("saveCSV: cannot create dir "
                + parent.string() + ": " + ec.message());
        }
    }
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw IOException("saveCSV: failed to open " + path);
    }
    ofs << "x,y,z\n";
    for (const auto& p : cloud->points) {
        ofs << p.x << ',' << p.y << ',' << p.z << '\n';
    }
    Logger::instance().info("Saved CSV: " + path);
}

// 功能：加载PLY文件，输入：path，返回：点云指针
PointCloudIO::CloudPtr PointCloudIO::loadPLY(const std::string& path) {
    auto fsPath = std::filesystem::u8path(path);
    if (!std::filesystem::exists(fsPath)) {
        throw IOException("loadPLY: file not found: " + path);
    }
    std::string actualPath = copyToTempIfNeeded(path, ".ply");
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    int ret = pcl::io::loadPLYFile(actualPath, *cloud);
    removeTempIfUsed(path, actualPath);
    if (ret != 0) {
        throw IOException("loadPLY: failed to read " + path);
    }
    Logger::instance().info("Loaded PLY: " + path
        + " (" + std::to_string(cloud->size()) + " pts)");
    return cloud;
}

// 功能：加载PCD文件，输入：path，返回：点云指针
PointCloudIO::CloudPtr PointCloudIO::loadPCD(const std::string& path) {
    auto fsPath = std::filesystem::u8path(path);
    if (!std::filesystem::exists(fsPath)) {
        throw IOException("loadPCD: file not found: " + path);
    }
    std::string actualPath = copyToTempIfNeeded(path, ".pcd");
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    int ret = pcl::io::loadPCDFile(actualPath, *cloud);
    removeTempIfUsed(path, actualPath);
    if (ret != 0) {
        throw IOException("loadPCD: failed to read " + path);
    }
    Logger::instance().info("Loaded PCD: " + path
        + " (" + std::to_string(cloud->size()) + " pts)");
    return cloud;
}

// 功能：根据扩展名自动加载点云，输入：path，返回：点云指针
PointCloudIO::CloudPtr PointCloudIO::load(const std::string& path) {
    auto ext = std::filesystem::u8path(path).extension().string();
    // 统一小写
    for (auto& ch : ext) ch = static_cast<char>(std::tolower(
        static_cast<unsigned char>(ch)));
    if (ext == ".ply") return loadPLY(path);
    if (ext == ".pcd") return loadPCD(path);
    throw IOException("load: unsupported extension '" + ext
        + "' for " + path);
}

} // namespace tp