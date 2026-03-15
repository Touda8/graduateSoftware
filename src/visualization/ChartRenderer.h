#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>
#include <QPixmap>
#include <QImage>

namespace tp {

struct BallResult;  // forward declaration

class ChartRenderer {
public:
    // JET colormap helper: normVal in [0,1] -> BGR color
    static cv::Scalar jetColor(double normVal);

    // cv::Mat (CV_8UC3 BGR) -> QPixmap
    static QPixmap matToPixmap(const cv::Mat& img);

    // Bar chart of ball heights by order number
    static cv::Mat renderBarChart(
        const std::vector<double>& heights,
        const std::vector<int>& orders,
        int width = 600, int height = 400);

    // Histogram of height distribution
    static cv::Mat renderHistogram(
        const std::vector<double>& values,
        int bins = 20,
        int width = 600, int height = 400);

    // Ball segmentation overlay: green circles + red centroids on grayscale image
    static cv::Mat renderBallOverlay(
        const cv::Mat& grayImage,
        const std::vector<BallResult>& balls);

    // Ball ordering overlay: colored dots by row + white order numbers
    static cv::Mat renderOrderOverlay(
        const cv::Mat& grayImage,
        const std::vector<BallResult>& balls,
        int numRows);
};

} // namespace tp
