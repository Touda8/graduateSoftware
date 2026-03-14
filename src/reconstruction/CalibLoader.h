#pragma once

#include <string>
#include <opencv2/core.hpp>
#include "common/interfaces.h"
#include "common/Exceptions.h"

namespace tp {

// 双投影仪标定数据
struct DualCalibData {
    cv::Mat allK_i1;  // 投影仪1系数矩阵 5013504×3
    cv::Mat allK_i2;  // 投影仪2系数矩阵 5013504×3
    cv::Mat X;        // 世界坐标X 2048×2448
    cv::Mat Y;        // 世界坐标Y 2048×2448
    int imgRows = 0;  // 2048
    int imgCols = 0;  // 2448
    bool isValid = false;
};

class CalibLoader {
public:
    // 从目录加载标定数据（优先读 raw/ 子目录）
    CalibData load(const std::string& dir);

    // 加载双投影仪标定数据
    DualCalibData loadDual(const std::string& rawDir);

private:
    // 读取单个 .raw 文件为 cv::Mat (CV_64F)
    cv::Mat readRaw(const std::string& path);
};

} // namespace tp