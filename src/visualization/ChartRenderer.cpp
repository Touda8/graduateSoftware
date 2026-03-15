#include "visualization/ChartRenderer.h"
#include "measurement/BGADetector.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <cstdio>

namespace tp {

// ---------------------------------------------------------------------------
// jetColor: map normVal in [0,1] to JET BGR
//   0.00 -> blue   (255,  0,  0) BGR
//   0.25 -> cyan   (255,255,  0)
//   0.50 -> green  (  0,255,  0)
//   0.75 -> yellow (  0,255,255)
//   1.00 -> red    (  0,  0,255)
// ---------------------------------------------------------------------------
cv::Scalar ChartRenderer::jetColor(double normVal)
{
    double v = std::clamp(normVal, 0.0, 1.0);

    double r = 0.0, g = 0.0, b = 0.0;

    if (v < 0.25) {
        // blue -> cyan: B=255, G ramps 0->255, R=0
        double t = v / 0.25;
        b = 255.0;
        g = 255.0 * t;
        r = 0.0;
    } else if (v < 0.5) {
        // cyan -> green: B ramps 255->0, G=255, R=0
        double t = (v - 0.25) / 0.25;
        b = 255.0 * (1.0 - t);
        g = 255.0;
        r = 0.0;
    } else if (v < 0.75) {
        // green -> yellow: B=0, G=255, R ramps 0->255
        double t = (v - 0.5) / 0.25;
        b = 0.0;
        g = 255.0;
        r = 255.0 * t;
    } else {
        // yellow -> red: B=0, G ramps 255->0, R=255
        double t = (v - 0.75) / 0.25;
        b = 0.0;
        g = 255.0 * (1.0 - t);
        r = 255.0;
    }

    // Return BGR
    return cv::Scalar(
        static_cast<int>(std::round(b)),
        static_cast<int>(std::round(g)),
        static_cast<int>(std::round(r)));
}

// ---------------------------------------------------------------------------
// matToPixmap: CV_8UC3 BGR -> QPixmap via QImage::Format_RGB888
// ---------------------------------------------------------------------------
QPixmap ChartRenderer::matToPixmap(const cv::Mat& img)
{
    if (img.empty()) {
        return QPixmap();
    }

    cv::Mat rgb;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

    QImage qimg(rgb.data,
                rgb.cols,
                rgb.rows,
                static_cast<int>(rgb.step),
                QImage::Format_RGB888);

    // Deep-copy so the QPixmap owns the data after rgb goes out of scope
    return QPixmap::fromImage(qimg.copy());
}

// ---------------------------------------------------------------------------
// renderBarChart
// ---------------------------------------------------------------------------
cv::Mat ChartRenderer::renderBarChart(
    const std::vector<double>& heights,
    const std::vector<int>& orders,
    int width, int height)
{
    constexpr int MARGIN_LEFT   = 70;
    constexpr int MARGIN_RIGHT  = 20;
    constexpr int MARGIN_TOP    = 40;
    constexpr int MARGIN_BOTTOM = 50;
    constexpr int Y_TICK_COUNT  = 5;

    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    if (heights.empty()) {
        return canvas;
    }

    const int plotW = width  - MARGIN_LEFT - MARGIN_RIGHT;
    const int plotH = height - MARGIN_TOP  - MARGIN_BOTTOM;
    const int n     = static_cast<int>(heights.size());

    // Y-axis range
    double minH = *std::min_element(heights.begin(), heights.end());
    double maxH = *std::max_element(heights.begin(), heights.end());
    if (std::abs(maxH - minH) < 1e-9) {
        minH -= 0.5;
        maxH += 0.5;
    }
    double rangeH  = maxH - minH;
    double padY    = rangeH * 0.1;
    double yMin    = minH - padY;
    double yMax    = maxH + padY;
    double yRange  = yMax - yMin;

    // Mean height
    double meanH = std::accumulate(heights.begin(), heights.end(), 0.0)
                   / static_cast<double>(n);

    // Lambda: value -> pixel Y
    auto valToY = [&](double v) -> int {
        return MARGIN_TOP + plotH
               - static_cast<int>((v - yMin) / yRange * plotH);
    };

    // --- Grid lines & Y-axis labels ---
    const cv::Scalar gridColor(210, 210, 210);
    for (int i = 0; i <= Y_TICK_COUNT; ++i) {
        double val = yMin + yRange * i / Y_TICK_COUNT;
        int py = valToY(val);
        cv::line(canvas,
                 cv::Point(MARGIN_LEFT, py),
                 cv::Point(MARGIN_LEFT + plotW, py),
                 gridColor, 1);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << val;
        cv::putText(canvas, oss.str(),
                    cv::Point(5, py + 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    // --- Axes ---
    cv::line(canvas,
             cv::Point(MARGIN_LEFT, MARGIN_TOP),
             cv::Point(MARGIN_LEFT, MARGIN_TOP + plotH),
             cv::Scalar(0, 0, 0), 1);
    cv::line(canvas,
             cv::Point(MARGIN_LEFT, MARGIN_TOP + plotH),
             cv::Point(MARGIN_LEFT + plotW, MARGIN_TOP + plotH),
             cv::Scalar(0, 0, 0), 1);

    // --- Bars ---
    double barSlotW = static_cast<double>(plotW) / n;
    double barW     = std::max(barSlotW * 0.7, 1.0);

    for (int i = 0; i < n; ++i) {
        double normVal = (heights[i] - minH) / rangeH;
        cv::Scalar color = jetColor(normVal);

        int cx = MARGIN_LEFT + static_cast<int>(barSlotW * (i + 0.5));
        int barHalf = static_cast<int>(barW / 2.0);
        int x1 = cx - barHalf;
        int x2 = cx + barHalf;
        int yTop = valToY(heights[i]);
        int yBot = valToY(yMin);

        // Filled bar
        cv::rectangle(canvas,
                      cv::Point(x1, yTop),
                      cv::Point(x2, yBot),
                      color, cv::FILLED);
        // 1px black border
        cv::rectangle(canvas,
                      cv::Point(x1, yTop),
                      cv::Point(x2, yBot),
                      cv::Scalar(0, 0, 0), 1);

        // X-axis label (order number)
        int ordIdx = (i < static_cast<int>(orders.size())) ? orders[i] : i + 1;
        std::string label = std::to_string(ordIdx);

        if (n <= 30) {
            cv::putText(canvas, label,
                        cv::Point(cx - 5, MARGIN_TOP + plotH + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3,
                        cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        } else if (i % 5 == 0) {
            // Show every 5th label when there are too many bars
            cv::putText(canvas, label,
                        cv::Point(cx - 5, MARGIN_TOP + plotH + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3,
                        cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        }
    }

    // --- Mean line (red dashed) ---
    int meanY = valToY(meanH);
    constexpr int DASH_LEN = 8;
    constexpr int GAP_LEN  = 4;
    for (int x = MARGIN_LEFT; x < MARGIN_LEFT + plotW; x += DASH_LEN + GAP_LEN) {
        int x2 = std::min(x + DASH_LEN, MARGIN_LEFT + plotW);
        cv::line(canvas,
                 cv::Point(x, meanY),
                 cv::Point(x2, meanY),
                 cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }

    // Mean label
    {
        std::ostringstream oss;
        oss << "mean=" << std::fixed << std::setprecision(3) << meanH;
        cv::putText(canvas, oss.str(),
                    cv::Point(MARGIN_LEFT + plotW - 100, meanY - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }

    // --- Title ---
    {
        int baseline = 0;
        std::string title = "Ball Height (mm)";
        cv::Size textSz = cv::getTextSize(title,
                                           cv::FONT_HERSHEY_SIMPLEX,
                                           0.55, 1, &baseline);
        int tx = (width - textSz.width) / 2;
        cv::putText(canvas, title,
                    cv::Point(tx, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    return canvas;
}

// ---------------------------------------------------------------------------
// renderHistogram
// ---------------------------------------------------------------------------
cv::Mat ChartRenderer::renderHistogram(
    const std::vector<double>& values,
    int bins,
    int width, int height)
{
    constexpr int MARGIN_LEFT   = 70;
    constexpr int MARGIN_RIGHT  = 20;
    constexpr int MARGIN_TOP    = 40;
    constexpr int MARGIN_BOTTOM = 50;

    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    if (values.empty() || bins <= 0) {
        return canvas;
    }

    const int plotW = width  - MARGIN_LEFT - MARGIN_RIGHT;
    const int plotH = height - MARGIN_TOP  - MARGIN_BOTTOM;

    double minV = *std::min_element(values.begin(), values.end());
    double maxV = *std::max_element(values.begin(), values.end());
    if (std::abs(maxV - minV) < 1e-9) {
        minV -= 0.5;
        maxV += 0.5;
    }
    double binWidth = (maxV - minV) / bins;

    // Compute bin counts
    std::vector<int> counts(bins, 0);
    for (double v : values) {
        int idx = static_cast<int>((v - minV) / binWidth);
        if (idx >= bins) idx = bins - 1;
        if (idx < 0)    idx = 0;
        counts[idx]++;
    }

    int maxCount = *std::max_element(counts.begin(), counts.end());
    if (maxCount == 0) maxCount = 1;

    // Statistics
    double meanV = std::accumulate(values.begin(), values.end(), 0.0)
                   / static_cast<double>(values.size());
    double sumSq = 0.0;
    for (double v : values) {
        sumSq += (v - meanV) * (v - meanV);
    }
    double stdV = std::sqrt(sumSq / static_cast<double>(values.size()));

    // --- Axes ---
    cv::line(canvas,
             cv::Point(MARGIN_LEFT, MARGIN_TOP),
             cv::Point(MARGIN_LEFT, MARGIN_TOP + plotH),
             cv::Scalar(0, 0, 0), 1);
    cv::line(canvas,
             cv::Point(MARGIN_LEFT, MARGIN_TOP + plotH),
             cv::Point(MARGIN_LEFT + plotW, MARGIN_TOP + plotH),
             cv::Scalar(0, 0, 0), 1);

    // --- Bars ---
    double barSlotW = static_cast<double>(plotW) / bins;
    const cv::Scalar barColor(180, 60, 40);  // blue-ish BGR

    for (int i = 0; i < bins; ++i) {
        double barH = static_cast<double>(counts[i]) / maxCount * plotH;
        int x1 = MARGIN_LEFT + static_cast<int>(barSlotW * i) + 1;
        int x2 = MARGIN_LEFT + static_cast<int>(barSlotW * (i + 1)) - 1;
        int yTop = MARGIN_TOP + plotH - static_cast<int>(barH);
        int yBot = MARGIN_TOP + plotH;

        cv::rectangle(canvas,
                      cv::Point(x1, yTop),
                      cv::Point(x2, yBot),
                      barColor, cv::FILLED);
        cv::rectangle(canvas,
                      cv::Point(x1, yTop),
                      cv::Point(x2, yBot),
                      cv::Scalar(0, 0, 0), 1);
    }

    // --- X-axis labels (show ~5 tick marks) ---
    {
        int tickCount = std::min(bins, 5);
        int step = std::max(bins / tickCount, 1);
        for (int i = 0; i <= bins; i += step) {
            double val = minV + binWidth * i;
            int px = MARGIN_LEFT + static_cast<int>(barSlotW * i);

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << val;
            cv::putText(canvas, oss.str(),
                        cv::Point(px - 10, MARGIN_TOP + plotH + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3,
                        cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        }
    }

    // --- Y-axis labels ---
    {
        constexpr int Y_TICK_COUNT = 5;
        for (int i = 0; i <= Y_TICK_COUNT; ++i) {
            int countVal = maxCount * i / Y_TICK_COUNT;
            int py = MARGIN_TOP + plotH
                     - static_cast<int>(static_cast<double>(countVal) / maxCount * plotH);

            cv::putText(canvas, std::to_string(countVal),
                        cv::Point(5, py + 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35,
                        cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

            cv::line(canvas,
                     cv::Point(MARGIN_LEFT, py),
                     cv::Point(MARGIN_LEFT + plotW, py),
                     cv::Scalar(210, 210, 210), 1);
        }
    }

    // --- Title ---
    {
        int baseline = 0;
        std::string title = "Height Distribution";
        cv::Size textSz = cv::getTextSize(title,
                                           cv::FONT_HERSHEY_SIMPLEX,
                                           0.55, 1, &baseline);
        int tx = (width - textSz.width) / 2;
        cv::putText(canvas, title,
                    cv::Point(tx, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    // --- Stats text box (top-right) ---
    {
        std::ostringstream ossN, ossM, ossS;
        ossN << "N = " << values.size();
        ossM << "mean = " << std::fixed << std::setprecision(3) << meanV;
        ossS << "std  = " << std::fixed << std::setprecision(3) << stdV;

        int boxX = width - MARGIN_RIGHT - 140;
        int boxY = MARGIN_TOP + 5;

        // Semi-transparent background
        cv::rectangle(canvas,
                      cv::Point(boxX - 5, boxY - 12),
                      cv::Point(boxX + 135, boxY + 42),
                      cv::Scalar(245, 245, 245), cv::FILLED);
        cv::rectangle(canvas,
                      cv::Point(boxX - 5, boxY - 12),
                      cv::Point(boxX + 135, boxY + 42),
                      cv::Scalar(180, 180, 180), 1);

        cv::putText(canvas, ossN.str(),
                    cv::Point(boxX, boxY + 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        cv::putText(canvas, ossM.str(),
                    cv::Point(boxX, boxY + 17),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        cv::putText(canvas, ossS.str(),
                    cv::Point(boxX, boxY + 32),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    return canvas;
}

// ---------------------------------------------------------------------------
// renderBallOverlay
// ---------------------------------------------------------------------------
cv::Mat ChartRenderer::renderBallOverlay(
    const cv::Mat& grayImage,
    const std::vector<BallResult>& balls)
{
    cv::Mat canvas;
    if (grayImage.channels() == 1) {
        cv::cvtColor(grayImage, canvas, cv::COLOR_GRAY2BGR);
    } else {
        canvas = grayImage.clone();
    }

    const cv::Scalar green(0, 255, 0);
    const cv::Scalar red(0, 0, 255);
    constexpr int MARKER_SIZE = 10;

    for (const auto& ball : balls) {
        if (!ball.success) {
            continue;
        }

        cv::Point center(static_cast<int>(std::round(ball.xc)),
                         static_cast<int>(std::round(ball.yc)));
        int r = static_cast<int>(std::round(ball.radius));

        // Green circle outline
        cv::circle(canvas, center, r, green, 2, cv::LINE_AA);

        // Red cross marker at centroid
        cv::drawMarker(canvas, center, red,
                       cv::MARKER_CROSS, MARKER_SIZE, 2, cv::LINE_AA);
    }

    return canvas;
}

// ---------------------------------------------------------------------------
// renderOrderOverlay
// ---------------------------------------------------------------------------
cv::Mat ChartRenderer::renderOrderOverlay(
    const cv::Mat& grayImage,
    const std::vector<BallResult>& balls,
    int numRows)
{
    cv::Mat canvas;
    if (grayImage.channels() == 1) {
        cv::cvtColor(grayImage, canvas, cv::COLOR_GRAY2BGR);
    } else {
        canvas = grayImage.clone();
    }

    if (numRows <= 0) {
        numRows = 1;
    }

    // Generate row-color palette via HSV
    std::vector<cv::Scalar> palette(numRows);
    for (int r = 0; r < numRows; ++r) {
        int hue = static_cast<int>(r * 180.0 / numRows);
        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 220, 230));
        cv::Mat bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
        auto px = bgr.at<cv::Vec3b>(0, 0);
        palette[r] = cv::Scalar(px[0], px[1], px[2]);
    }

    const cv::Scalar white(255, 255, 255);
    constexpr int DOT_RADIUS = 8;

    for (const auto& ball : balls) {
        cv::Point center(static_cast<int>(std::round(ball.xc)),
                         static_cast<int>(std::round(ball.yc)));

        int rowIdx = std::clamp(ball.row, 0, numRows - 1);
        cv::Scalar color = palette[rowIdx];

        // Filled circle with row-specific color
        cv::circle(canvas, center, DOT_RADIUS, color, cv::FILLED, cv::LINE_AA);

        // White order number text near center
        std::string label = std::to_string(ball.order);
        cv::putText(canvas, label,
                    cv::Point(center.x + DOT_RADIUS + 2, center.y + 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    white, 1, cv::LINE_AA);
    }

    return canvas;
}

// ---------------------------------------------------------------------------
// renderCoplanarityLineChart — N 次共面度重复测量折线图
// ---------------------------------------------------------------------------
cv::Mat ChartRenderer::renderCoplanarityLineChart(
    const std::vector<double>& coplanarities,
    int width, int height)
{
    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    const int n = static_cast<int>(coplanarities.size());
    if (n == 0) return canvas;

    // Margins
    constexpr int marginL = 80, marginR = 30, marginT = 40, marginB = 50;
    int plotW = width - marginL - marginR;
    int plotH = height - marginT - marginB;

    // Y range
    double minVal = *std::min_element(coplanarities.begin(), coplanarities.end());
    double maxVal = *std::max_element(coplanarities.begin(), coplanarities.end());
    double range = maxVal - minVal;
    if (range < 1e-9) range = std::max(maxVal * 0.1, 0.001);
    double yLo = minVal - range * 0.15;
    double yHi = maxVal + range * 0.15;
    double yRange = yHi - yLo;

    // Mean line
    double sum = 0;
    for (auto v : coplanarities) sum += v;
    double mean = sum / n;

    // Std dev
    double sqSum = 0;
    for (auto v : coplanarities) sqSum += (v - mean) * (v - mean);
    double stdDev = (n > 1) ? std::sqrt(sqSum / (n - 1)) : 0;

    auto toScreenX = [&](int idx) -> int {
        if (n == 1) return marginL + plotW / 2;
        return marginL + static_cast<int>(idx * plotW / (n - 1.0));
    };
    auto toScreenY = [&](double val) -> int {
        return marginT + static_cast<int>((yHi - val) / yRange * plotH);
    };

    // Draw axes
    const cv::Scalar black(0, 0, 0);
    const cv::Scalar gray(200, 200, 200);
    cv::line(canvas, {marginL, marginT}, {marginL, height - marginB}, black, 1);
    cv::line(canvas, {marginL, height - marginB}, {width - marginR, height - marginB}, black, 1);

    // Y grid lines (5 ticks)
    for (int i = 0; i <= 4; ++i) {
        double val = yLo + yRange * i / 4.0;
        int sy = toScreenY(val);
        cv::line(canvas, {marginL, sy}, {width - marginR, sy}, gray, 1);
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.4f", val);
        cv::putText(canvas, buf, {5, sy + 4}, cv::FONT_HERSHEY_SIMPLEX, 0.35, black, 1, cv::LINE_AA);
    }

    // X labels
    for (int i = 0; i < n; ++i) {
        int sx = toScreenX(i);
        cv::line(canvas, {sx, height - marginB}, {sx, height - marginB + 4}, black, 1);
        std::string label = std::to_string(i + 1);
        cv::putText(canvas, label, {sx - 4, height - marginB + 18},
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, black, 1, cv::LINE_AA);
    }

    // Mean line (dashed effect)
    {
        int my = toScreenY(mean);
        const cv::Scalar orange(0, 140, 255);
        for (int x = marginL; x < width - marginR; x += 8) {
            int x2 = std::min(x + 4, width - marginR);
            cv::line(canvas, {x, my}, {x2, my}, orange, 1);
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "Mean=%.4f", mean);
        cv::putText(canvas, buf, {width - marginR - 120, my - 6},
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, orange, 1, cv::LINE_AA);
    }

    // Draw polyline
    const cv::Scalar blue(200, 80, 0);
    std::vector<cv::Point> pts(n);
    for (int i = 0; i < n; ++i) {
        pts[i] = {toScreenX(i), toScreenY(coplanarities[i])};
    }
    cv::polylines(canvas, pts, false, blue, 2, cv::LINE_AA);

    // Draw data points
    const cv::Scalar red(0, 0, 220);
    for (int i = 0; i < n; ++i) {
        cv::circle(canvas, pts[i], 4, red, cv::FILLED, cv::LINE_AA);
    }

    // Title
    cv::putText(canvas, "BGA Coplanarity Repeated Measurement",
                {marginL + 10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.50, black, 1, cv::LINE_AA);

    // Stats box
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "N=%d  Mean=%.4f mm  Std=%.4f mm", n, mean, stdDev);
        cv::putText(canvas, buf, {marginL + 10, height - 8},
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, black, 1, cv::LINE_AA);
    }

    return canvas;
}

} // namespace tp
