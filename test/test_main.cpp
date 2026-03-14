#define _USE_MATH_DEFINES
#include "TestFramework.h"
#include <clocale>

// 包含所有测试文件
#include "unit/TestPhaseDecoder.cpp"
#include "unit/TestBGADetector.cpp"
#include "unit/TestPointCloudFilter.cpp"
#include "integration/TestReconstructionPipeline.cpp"
#include "integration/TestCrashPrevention.cpp"

int main() {
    // Windows: 让 CRT 使用 UTF-8 编码，确保中文路径正常工作
    std::setlocale(LC_ALL, ".UTF-8");
    std::cout << "=== graduateSoftware Test Suite ===\n\n";
    return testing::runAll();
}
