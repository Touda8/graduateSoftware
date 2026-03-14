// TestPhaseDecoder.cpp - PhaseDecoder 单元测试
#include "TestFramework.h"
#include "reconstruction/PhaseDecoder.h"
#include "common/Exceptions.h"
#include <opencv2/core.hpp>
#include <cmath>

// 辅助函数：生成合成N步相移条纹图
std::vector<cv::Mat> makeSyntheticFringes(int rows, int cols,
                                          double period, int steps) {
    std::vector<cv::Mat> imgs(steps);
    for (int s = 0; s < steps; ++s) {
        imgs[s] = cv::Mat(rows, cols, CV_64F);
        double delta = 2.0 * M_PI * s / steps;
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                imgs[s].at<double>(r, c) =
                    128.0 + 100.0 * std::cos(2.0 * M_PI * c / period + delta);
    }
    return imgs;
}

TEST(PhaseDecoder, NormalDecode4Step) {
    auto imgs = makeSyntheticFringes(200, 200, 40.0, 4);
    tp::PhaseDecoder decoder;
    cv::Mat phase = decoder.calcPhaseMatrix(imgs, 4);

    ASSERT_EQ(phase.rows, 200);
    ASSERT_EQ(phase.cols, 200);
    ASSERT_EQ(phase.type(), CV_64F);

    // 验证所有像素值在 [-pi, pi] 范围内
    for (int r = 0; r < phase.rows; ++r) {
        for (int c = 0; c < phase.cols; ++c) {
            double v = phase.at<double>(r, c);
            ASSERT_TRUE(v >= -M_PI - 1e-6 && v <= M_PI + 1e-6);
        }
    }
}

TEST(PhaseDecoder, NormalDecode8Step) {
    auto imgs = makeSyntheticFringes(100, 100, 25.0, 8);
    tp::PhaseDecoder decoder;
    cv::Mat phase = decoder.calcPhaseMatrix(imgs, 8);

    ASSERT_EQ(phase.rows, 100);
    ASSERT_EQ(phase.cols, 100);
    for (int r = 0; r < phase.rows; ++r) {
        for (int c = 0; c < phase.cols; ++c) {
            double v = phase.at<double>(r, c);
            ASSERT_TRUE(v >= -M_PI - 1e-6 && v <= M_PI + 1e-6);
        }
    }
}

TEST(PhaseDecoder, EmptyInput) {
    tp::PhaseDecoder decoder;
    std::vector<cv::Mat> empty;
    ASSERT_THROW(decoder.calcPhaseMatrix(empty, 4),
                 tp::ImageLoadException);
}

TEST(PhaseDecoder, InsufficientImages) {
    auto imgs = makeSyntheticFringes(50, 50, 20.0, 2);
    tp::PhaseDecoder decoder;
    ASSERT_THROW(decoder.calcPhaseMatrix(imgs, 4),
                 tp::ImageLoadException);
}

TEST(PhaseDecoder, FringeModulation) {
    auto imgs = makeSyntheticFringes(100, 100, 30.0, 4);
    tp::PhaseDecoder decoder;
    cv::Mat mod = decoder.calcFringeModulation(imgs, 4);

    ASSERT_EQ(mod.rows, 100);
    ASSERT_EQ(mod.cols, 100);
    ASSERT_FALSE(mod.empty());

    // 合成条纹图的调制度应 > 0
    double minVal = 0, maxVal = 0;
    cv::minMaxLoc(mod, &minVal, &maxVal);
    ASSERT_GT(maxVal, 0.0);
}

TEST(PhaseDecoder, FringeModulationEmptyInput) {
    tp::PhaseDecoder decoder;
    std::vector<cv::Mat> empty;
    ASSERT_THROW(decoder.calcFringeModulation(empty, 4),
                 tp::ImageLoadException);
}

