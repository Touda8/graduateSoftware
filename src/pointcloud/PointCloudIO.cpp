#include "pointcloud/PointCloudIO.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <filesystem>

namespace tp {

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
    if (!std::filesystem::exists(path)) {
        throw IOException("loadPLY: file not found: " + path);
    }
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPLYFile(path, *cloud) != 0) {
        throw IOException("loadPLY: failed to read " + path);
    }
    Logger::instance().info("Loaded PLY: " + path
        + " (" + std::to_string(cloud->size()) + " pts)");
    return cloud;
}

// 功能：加载PCD文件，输入：path，返回：点云指针
PointCloudIO::CloudPtr PointCloudIO::loadPCD(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        throw IOException("loadPCD: file not found: " + path);
    }
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(path, *cloud) != 0) {
        throw IOException("loadPCD: failed to read " + path);
    }
    Logger::instance().info("Loaded PCD: " + path
        + " (" + std::to_string(cloud->size()) + " pts)");
    return cloud;
}

// 功能：根据扩展名自动加载点云，输入：path，返回：点云指针
PointCloudIO::CloudPtr PointCloudIO::load(const std::string& path) {
    auto ext = std::filesystem::path(path).extension().string();
    // 统一小写
    for (auto& ch : ext) ch = static_cast<char>(std::tolower(
        static_cast<unsigned char>(ch)));
    if (ext == ".ply") return loadPLY(path);
    if (ext == ".pcd") return loadPCD(path);
    throw IOException("load: unsupported extension '" + ext
        + "' for " + path);
}

} // namespace tp