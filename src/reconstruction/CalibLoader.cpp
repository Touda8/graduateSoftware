#include "reconstruction/CalibLoader.h"
#include "common/Logger.h"
#include <fstream>
#include <filesystem>
#include <cstdint>

namespace tp {

cv::Mat CalibLoader::readRaw(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        throw CalibException("readRaw: cannot open " + path);
    }
    int32_t rows = 0, cols = 0;
    ifs.read(reinterpret_cast<char*>(&rows), sizeof(int32_t));
    ifs.read(reinterpret_cast<char*>(&cols), sizeof(int32_t));
    if (rows <= 0 || cols <= 0) {
        throw CalibException("readRaw: invalid dims " +
            std::to_string(rows) + "x" + std::to_string(cols) +
            " in " + path);
    }
    cv::Mat mat(rows, cols, CV_64F);
    const auto dataSize =
        static_cast<std::streamsize>(rows) * cols * sizeof(double);
    ifs.read(reinterpret_cast<char*>(mat.data), dataSize);
    if (!ifs) {
        throw CalibException("readRaw: incomplete read from " + path);
    }
    Logger::instance().info("readRaw: " + path + " [" +
        std::to_string(rows) + " x " + std::to_string(cols) + "]");
    return mat;
}

CalibData CalibLoader::load(const std::string& dir) {
    namespace fs = std::filesystem;
    const fs::path rawDir = fs::path(dir) / "raw";

    if (!fs::is_directory(rawDir)) {
        throw CalibException(
            "CalibLoader::load: raw/ not found in " + dir +
            ". Run convert_calib.m first.");
    }

    CalibData calib;

    calib.X = readRaw((rawDir / "X.raw").string());
    calib.Y = readRaw((rawDir / "Y.raw").string());

    calib.allK   = readRaw((rawDir / "allK1.raw").string());
    calib.allK_i = readRaw((rawDir / "allK_i1.raw").string());

    calib.isValid = !calib.allK.empty() && !calib.X.empty();

    Logger::instance().info("CalibLoader: loaded from " + dir +
        (calib.isValid ? " [valid]" : " [incomplete]"));
    return calib;
}

// 功能：加载双投影仪标定数据，输入：rawDir路径，返回：DualCalibData
DualCalibData CalibLoader::loadDual(const std::string& rawDir) {
    namespace fs = std::filesystem;
    if (!fs::is_directory(rawDir)) {
        throw CalibException(
            "CalibLoader::loadDual: directory not found: " + rawDir);
    }

    DualCalibData dc;
    dc.X = readRaw((fs::path(rawDir) / "X.raw").string());
    dc.Y = readRaw((fs::path(rawDir) / "Y.raw").string());
    dc.allK_i1 = readRaw(
        (fs::path(rawDir) / "allK_i1.raw").string());
    dc.allK_i2 = readRaw(
        (fs::path(rawDir) / "allK_i2.raw").string());

    dc.imgRows = dc.X.rows;
    dc.imgCols = dc.X.cols;
    dc.isValid = !dc.X.empty() && !dc.allK_i1.empty()
        && (dc.imgRows * dc.imgCols == dc.allK_i1.rows);

    Logger::instance().info("CalibLoader::loadDual: " + rawDir
        + " imgSize=" + std::to_string(dc.imgRows) + "x"
        + std::to_string(dc.imgCols)
        + (dc.isValid ? " [valid]" : " [invalid]"));
    return dc;
}

} // namespace tp