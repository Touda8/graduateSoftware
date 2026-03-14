#define _USE_MATH_DEFINES
#include "BGAMeasurePipeline.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace tp {

// 功能：两直线交点，输入：l1,l2 = (a,b,c)，ax+by+c=0
cv::Point2d BGAMeasurePipeline::lineIntersection(
    const cv::Vec3d& l1, const cv::Vec3d& l2)
{
    double det = l1[0] * l2[1] - l1[1] * l2[0];
    if (std::abs(det) < 1e-10)
        throw std::runtime_error("lineIntersection: parallel lines");
    double x = (l1[1] * l2[2] - l2[1] * l1[2]) / det;
    double y = (l2[0] * l1[2] - l1[0] * l2[2]) / det;
    return {x, y};
}

// 功能：IRLS加权最小二乘直线拟合(Huber权重)
cv::Vec3d BGAMeasurePipeline::irlsLineFit(
    const std::vector<cv::Point2d>& pts, int maxIter, double sigma)
{
    if (pts.size() < 3)
        throw tp::MeasureException("irlsLineFit: need >= 3 points");

    const int n = static_cast<int>(pts.size());
    Eigen::MatrixXd A(n, 2);
    Eigen::VectorXd b(n), w(n);
    w.setOnes();

    // 判断主方向(水平/垂直)
    double xRange = 0, yRange = 0;
    for (const auto& p : pts) {
        xRange = std::max(xRange, std::abs(p.x - pts[0].x));
        yRange = std::max(yRange, std::abs(p.y - pts[0].y));
    }
    bool horizontal = (xRange >= yRange);

    Eigen::Vector2d sol;
    for (int iter = 0; iter < maxIter; ++iter) {
        for (int i = 0; i < n; ++i) {
            if (horizontal) {
                A(i, 0) = pts[i].x * w(i);
                A(i, 1) = w(i);
                b(i) = pts[i].y * w(i);
            } else {
                A(i, 0) = pts[i].y * w(i);
                A(i, 1) = w(i);
                b(i) = pts[i].x * w(i);
            }
        }
        sol = A.colPivHouseholderQr().solve(b);

        // 更新Huber权重
        for (int i = 0; i < n; ++i) {
            double residual = horizontal
                ? (pts[i].y - sol(0) * pts[i].x - sol(1))
                : (pts[i].x - sol(0) * pts[i].y - sol(1));
            double absR = std::abs(residual);
            w(i) = (absR <= sigma) ? 1.0 : sigma / absR;
        }
    }
    // 转为 ax+by+c=0 形式
    if (horizontal)
        return cv::Vec3d(-sol(0), 1.0, -sol(1)); // y=kx+b → -kx+y-b=0
    else
        return cv::Vec3d(1.0, -sol(0), -sol(1)); // x=ky+b → x-ky-b=0
}

// 功能：分层Otsu芯片区域定位 + 由内向外扫描 + IRLS边界拟合
ChipInfo BGAMeasurePipeline::localizeChip(
    const cv::Mat& gray, cv::Mat& maskOut)
{
    if (gray.empty())
        throw tp::ImageLoadException("localizeChip: empty image");

    cv::Mat img;
    if (gray.type() != CV_64F) gray.convertTo(img, CV_64F, 1.0 / 255.0);
    else img = gray.clone();
    const int h = img.rows, w = img.cols;

    // Level-1 全局Otsu
    cv::Mat gray8;
    img.convertTo(gray8, CV_8U, 255.0);
    double t1Raw = cv::threshold(gray8, cv::noArray(), 0, 255,
                                 cv::THRESH_BINARY | cv::THRESH_OTSU);
    double T1 = t1Raw / 255.0;

    // Level-2 暗区内二次Otsu
    cv::Mat darkMask = (img <= T1);
    std::vector<uchar> darkVals;
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            if (darkMask.at<uchar>(r, c))
                darkVals.push_back(gray8.at<uchar>(r, c));
    cv::Mat darkHist(darkVals);
    double t2Raw = cv::threshold(darkHist, cv::noArray(), 0, 255,
                                 cv::THRESH_BINARY | cv::THRESH_OTSU);
    double T2 = t2Raw / 255.0;

    // 芯片体掩膜
    cv::Mat chipBody;
    cv::threshold(gray8, chipBody, t2Raw, 255, cv::THRESH_BINARY_INV);

    // 腐蚀断开边缘暗角连接
    auto seBreak = cv::getStructuringElement(cv::MORPH_ELLIPSE, {7, 7});
    cv::erode(chipBody, chipBody, seBreak);

    // 清除接触边界的连通域
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(
        chipBody, labels, stats, centroids, 8, CV_32S);
    cv::Mat interior = cv::Mat::zeros(h, w, CV_8U);
    for (int i = 1; i < nLabels; ++i) {
        int x0 = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y0 = stats.at<int>(i, cv::CC_STAT_TOP);
        int bw = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int bh = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        // 排除触及图像边界的连通域
        if (x0 <= 0 || y0 <= 0 || x0 + bw >= w || y0 + bh >= h)
            continue;
        cv::Mat compMask = (labels == i);
        compMask.convertTo(compMask, CV_8U, 255);
        interior |= compMask;
    }

    // 形态学恢复 + 闭运算 + 填孔
    auto seRecover = cv::getStructuringElement(cv::MORPH_ELLIPSE, {9, 9});
    cv::dilate(interior, interior, seRecover);
    auto seClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, {21, 21});
    cv::morphologyEx(interior, interior, cv::MORPH_CLOSE, seClose);

    // 选择最大连通域作为芯片体
    cv::Mat labels2, stats2, cent2;
    int nL2 = cv::connectedComponentsWithStats(
        interior, labels2, stats2, cent2, 8, CV_32S);
    int bestIdx = -1, bestArea = 0;
    for (int i = 1; i < nL2; ++i) {
        int area = stats2.at<int>(i, cv::CC_STAT_AREA);
        if (area > bestArea) { bestArea = area; bestIdx = i; }
    }
    cv::Mat coarseMask = (labels2 == bestIdx);
    coarseMask.convertTo(coarseMask, CV_8U, 255);

    // 质心
    cv::Moments mom = cv::moments(coarseMask, true);
    int cx = static_cast<int>(mom.m10 / mom.m00);
    int cy = static_cast<int>(mom.m01 / mom.m00);

    // 粗掩膜的BoundingBox
    cv::Rect bbox = cv::boundingRect(coarseMask);
    double Tedge = T2 + (T1 - T2) * 0.3;
    constexpr int SEARCH_MARGIN = 20;

    // 由内向外扫描四条边
    auto scanEdge = [&](bool horiz, bool positive,
        int fixedStart, int fixedEnd,
        int scanStart, int scanLimit) -> std::vector<cv::Point2d>
    {
        std::vector<cv::Point2d> pts;
        for (int f = fixedStart; f <= fixedEnd; ++f) {
            int step = positive ? 1 : -1;
            for (int s = scanStart; s != scanLimit + step; s += step) {
                int r = horiz ? s : f;
                int c = horiz ? f : s;
                if (r < 0 || r >= h || c < 0 || c >= w) break;
                if (img.at<double>(r, c) > Tedge) {
                    pts.emplace_back(c, r);
                    break;
                }
            }
        }
        return pts;
    };

    int xTrim = static_cast<int>(bbox.width * 0.2);
    int yTrim = static_cast<int>(bbox.height * 0.2);
    // 上边: 中间60%列，从质心向上
    auto ptsTop = scanEdge(true, false,
        bbox.x + xTrim, bbox.x + bbox.width - xTrim,
        cy, std::max(0, bbox.y - SEARCH_MARGIN));
    // 下边
    auto ptsBot = scanEdge(true, true,
        bbox.x + xTrim, bbox.x + bbox.width - xTrim,
        cy, std::min(h - 1, bbox.y + bbox.height + SEARCH_MARGIN));
    // 左边
    auto ptsLeft = scanEdge(false, false,
        bbox.y + yTrim, bbox.y + bbox.height - yTrim,
        cx, std::max(0, bbox.x - SEARCH_MARGIN));
    // 右边
    auto ptsRight = scanEdge(false, true,
        bbox.y + yTrim, bbox.y + bbox.height - yTrim,
        cx, std::min(w - 1, bbox.x + bbox.width + SEARCH_MARGIN));

    constexpr int IRLS_ITER = 8;
    constexpr double IRLS_SIGMA = 1.5;
    cv::Vec3d lTop = irlsLineFit(ptsTop, IRLS_ITER, IRLS_SIGMA);
    cv::Vec3d lBot = irlsLineFit(ptsBot, IRLS_ITER, IRLS_SIGMA);
    cv::Vec3d lLeft = irlsLineFit(ptsLeft, IRLS_ITER, IRLS_SIGMA);
    cv::Vec3d lRight = irlsLineFit(ptsRight, IRLS_ITER, IRLS_SIGMA);

    ChipInfo info;
    info.lineTop = lTop; info.lineBot = lBot;
    info.lineLeft = lLeft; info.lineRight = lRight;
    info.corners[0] = lineIntersection(lTop, lLeft);    // 左上
    info.corners[1] = lineIntersection(lTop, lRight);   // 右上
    info.corners[2] = lineIntersection(lBot, lRight);   // 右下
    info.corners[3] = lineIntersection(lBot, lLeft);    // 左下

    // 生成精确矩形掩膜
    std::vector<cv::Point> polyPts;
    for (const auto& c : info.corners)
        polyPts.emplace_back(static_cast<int>(c.x), static_cast<int>(c.y));
    maskOut = cv::Mat::zeros(h, w, CV_8U);
    cv::fillConvexPoly(maskOut, polyPts, 255);
    return info;
}

// 功能：封装本体边界精化(Canny+迭代直线拟合)
cv::Rect BGAMeasurePipeline::refineBodyBoundary(
    const cv::Mat& gray, const cv::Mat& chipMask)
{
    if (gray.empty() || chipMask.empty())
        throw tp::ImageLoadException("refineBodyBoundary: empty input");

    // 芯片区域内Otsu分割
    cv::Mat masked;
    gray.copyTo(masked);
    masked.setTo(0, ~chipMask);

    cv::Mat gray8;
    if (masked.type() != CV_8U) masked.convertTo(gray8, CV_8U, 255.0);
    else gray8 = masked;

    double thr = cv::threshold(gray8, cv::noArray(), 0, 255,
                               cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Mat binary;
    cv::threshold(gray8, binary, thr, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(binary, chipMask, binary);

    // 闭运算 + 最大连通域
    auto se = cv::getStructuringElement(cv::MORPH_ELLIPSE, {11, 11});
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, se);
    cv::Mat labels, stats, cent;
    int nL = cv::connectedComponentsWithStats(
        binary, labels, stats, cent, 8, CV_32S);
    int bestIdx = -1, bestArea = 0;
    for (int i = 1; i < nL; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > bestArea) { bestArea = area; bestIdx = i; }
    }
    if (bestIdx < 0)
        throw std::runtime_error("refineBodyBoundary: no body found");

    int bx = stats.at<int>(bestIdx, cv::CC_STAT_LEFT);
    int by = stats.at<int>(bestIdx, cv::CC_STAT_TOP);
    int bw = stats.at<int>(bestIdx, cv::CC_STAT_WIDTH);
    int bh = stats.at<int>(bestIdx, cv::CC_STAT_HEIGHT);
    return cv::Rect(bx, by, bw, bh);
}

// 功能：确定四侧引脚搜索区域
std::vector<cv::Rect> BGAMeasurePipeline::defineSearchRegions(
    const ChipInfo& info, const cv::Size& imgSize)
{
    constexpr int PAD = 50; // 搜索区域向外扩展的像素

    double xMin = info.corners[0].x, xMax = info.corners[0].x;
    double yMin = info.corners[0].y, yMax = info.corners[0].y;
    for (const auto& c : info.corners) {
        xMin = std::min(xMin, c.x); xMax = std::max(xMax, c.x);
        yMin = std::min(yMin, c.y); yMax = std::max(yMax, c.y);
    }
    int ix = static_cast<int>(xMin), iy = static_cast<int>(yMin);
    int iw = static_cast<int>(xMax - xMin);
    int ih = static_cast<int>(yMax - yMin);

    std::vector<cv::Rect> regions(4);
    // 上侧
    regions[0] = cv::Rect(ix, std::max(0, iy - PAD), iw, PAD)
                 & cv::Rect(0, 0, imgSize.width, imgSize.height);
    // 下侧
    regions[1] = cv::Rect(ix, iy + ih, iw, PAD)
                 & cv::Rect(0, 0, imgSize.width, imgSize.height);
    // 左侧
    regions[2] = cv::Rect(std::max(0, ix - PAD), iy, PAD, ih)
                 & cv::Rect(0, 0, imgSize.width, imgSize.height);
    // 右侧
    regions[3] = cv::Rect(ix + iw, iy, PAD, ih)
                 & cv::Rect(0, 0, imgSize.width, imgSize.height);
    return regions;
}

// 功能：几何过滤+引脚分割，输入：灰度图、搜索区域、本体矩形
std::vector<PinRegion> BGAMeasurePipeline::segmentPins(
    const cv::Mat& gray, const std::vector<cv::Rect>& regions,
    const cv::Rect& bodyRect)
{
    if (gray.empty())
        throw tp::ImageLoadException("segmentPins: empty image");

    constexpr double AREA_RATIO_MIN = 0.3;   // 相对中位数面积
    constexpr double AREA_RATIO_MAX = 3.0;
    constexpr double ASPECT_RATIO_MAX = 5.0;
    const std::array<std::string, 4> sideNames = {
        "top", "bottom", "left", "right"};

    cv::Mat gray8;
    if (gray.type() != CV_8U) gray.convertTo(gray8, CV_8U, 255.0);
    else gray8 = gray;

    std::vector<PinRegion> allPins;
    int pinId = 0;

    for (size_t s = 0; s < regions.size(); ++s) {
        const cv::Rect& roi = regions[s];
        if (roi.area() <= 0) continue;

        cv::Mat roiImg = gray8(roi);
        double thr = cv::threshold(roiImg, cv::noArray(), 0, 255,
                                   cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::Mat binary;
        cv::threshold(roiImg, binary, thr, 255, cv::THRESH_BINARY);

        // 形态学开运算去除噪点
        auto se = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, se);

        // 连通域分析
        cv::Mat labels, stats, cent;
        int nL = cv::connectedComponentsWithStats(
            binary, labels, stats, cent, 8, CV_32S);

        // 收集候选区域面积做中位数估计
        std::vector<int> areas;
        for (int i = 1; i < nL; ++i)
            areas.push_back(stats.at<int>(i, cv::CC_STAT_AREA));
        if (areas.empty()) continue;
        std::sort(areas.begin(), areas.end());
        double medArea = areas[areas.size() / 2];

        for (int i = 1; i < nL; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < medArea * AREA_RATIO_MIN ||
                area > medArea * AREA_RATIO_MAX) continue;

            int bw = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int bh = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            double aspect = static_cast<double>(
                std::max(bw, bh)) / std::max(1, std::min(bw, bh));
            if (aspect > ASPECT_RATIO_MAX) continue;

            PinRegion pin;
            pin.pinId = ++pinId;
            pin.side = sideNames[s];
            pin.bbox = cv::Rect(
                roi.x + stats.at<int>(i, cv::CC_STAT_LEFT),
                roi.y + stats.at<int>(i, cv::CC_STAT_TOP),
                bw, bh);
            pin.centroid = cv::Point2d(
                roi.x + cent.at<double>(i, 0),
                roi.y + cent.at<double>(i, 1));
            pin.area = area;
            cv::Mat compMask = (labels == i);
            compMask.convertTo(pin.mask, CV_8U, 255);
            // 偏移到全图坐标
            cv::Mat fullMask = cv::Mat::zeros(gray.size(), CV_8U);
            pin.mask.copyTo(fullMask(roi));
            pin.mask = fullMask;
            allPins.push_back(std::move(pin));
        }
    }
    return allPins;
}

// 功能：封装本体二阶曲面拟合，返回系数[s0..s5]和法向量
std::array<double, 6> BGAMeasurePipeline::fitBodySurface(
    const cv::Mat& X, const cv::Mat& Y, const cv::Mat& Z,
    const cv::Mat& bodyMask, Eigen::Vector3d& normalOut)
{
    if (X.empty() || Y.empty() || Z.empty() || bodyMask.empty())
        throw tp::MeasureException("fitBodySurface: empty input");

    // 腐蚀掩膜去除边缘杂点
    cv::Mat mask;
    auto se = cv::getStructuringElement(cv::MORPH_RECT, {50, 50});
    cv::erode(bodyMask, mask, se);

    // 收集有效点
    std::vector<double> xs, ys, zs;
    for (int r = 0; r < Z.rows; ++r) {
        for (int c = 0; c < Z.cols; ++c) {
            if (!mask.at<uchar>(r, c)) continue;
            double z = Z.at<double>(r, c);
            if (z == 0 || std::isnan(z)) continue;
            xs.push_back(X.at<double>(r, c));
            ys.push_back(Y.at<double>(r, c));
            zs.push_back(z);
        }
    }
    const int n = static_cast<int>(xs.size());
    if (n < 100)
        throw std::runtime_error("fitBodySurface: insufficient points");

    // Z = s0 + s1*X + s2*Y + s3*X² + s4*Y² + s5*XY
    Eigen::MatrixXd A(n, 6);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) {
        A(i, 0) = 1.0;
        A(i, 1) = xs[i];
        A(i, 2) = ys[i];
        A(i, 3) = xs[i] * xs[i];
        A(i, 4) = ys[i] * ys[i];
        A(i, 5) = xs[i] * ys[i];
        b(i) = zs[i];
    }
    Eigen::VectorXd sol = A.colPivHouseholderQr().solve(b);

    // 法向量: 从一阶系数 n ≈ (-s1, -s2, 1) 归一化
    normalOut = Eigen::Vector3d(-sol(1), -sol(2), 1.0).normalized();

    return {sol(0), sol(1), sol(2), sol(3), sol(4), sol(5)};
}

// 功能：引脚底部接触区提取(高度分层)
std::vector<PinBottomCloud> BGAMeasurePipeline::extractPinBottoms(
    const cv::Mat& X, const cv::Mat& Y, const cv::Mat& Z,
    const std::vector<PinRegion>& pins)
{
    constexpr double KZ = 2.0;   // 高度阈值系数

    std::vector<PinBottomCloud> result;
    result.reserve(pins.size());

    for (const auto& pin : pins) {
        PinBottomCloud pbc;
        pbc.pinId = pin.pinId;
        pbc.side = pin.side;

        // 提取引脚区域的3D点
        for (int r = pin.bbox.y;
             r < pin.bbox.y + pin.bbox.height && r < Z.rows; ++r) {
            for (int c = pin.bbox.x;
                 c < pin.bbox.x + pin.bbox.width && c < Z.cols; ++c) {
                if (r < 0 || c < 0) continue;
                if (!pin.mask.at<uchar>(r, c)) continue;
                double z = Z.at<double>(r, c);
                if (z == 0 || std::isnan(z)) continue;
                pbc.fullCloud.emplace_back(
                    X.at<double>(r, c), Y.at<double>(r, c), z);
            }
        }
        if (pbc.fullCloud.size() < 10) continue;

        // 高度直方图自适应阈值
        std::vector<double> zVals;
        zVals.reserve(pbc.fullCloud.size());
        for (const auto& p : pbc.fullCloud) zVals.push_back(p(2));
        std::sort(zVals.begin(), zVals.end());

        // 底部30%的点 → 统计
        int nLow = std::max(3, static_cast<int>(zVals.size() * 0.3));
        double zMedLow = zVals[nLow / 2];
        double sumSq = 0;
        for (int i = 0; i < nLow; ++i) {
            double d = zVals[i] - zMedLow;
            sumSq += d * d;
        }
        double sigmaLow = std::sqrt(sumSq / nLow);
        double zThresh = zMedLow + KZ * sigmaLow;

        for (const auto& p : pbc.fullCloud) {
            if (p(2) <= zThresh) pbc.bottomCloud.push_back(p);
        }
        result.push_back(std::move(pbc));
    }
    return result;
}

// 功能：法向量约束+空间连通性点云精化
void BGAMeasurePipeline::refineBottomClouds(
    std::vector<PinBottomCloud>& pins,
    const Eigen::Vector3d& bodyNormal)
{
    constexpr double THETA_THR = 30.0;    // 法向量夹角阈值(度)
    constexpr double CLUSTER_DIST = 0.3;  // 聚类距离(mm)
    constexpr int MIN_NEIGHBORS = 5;

    for (auto& pin : pins) {
        if (pin.bottomCloud.size() < MIN_NEIGHBORS) {
            pin.refinedCloud = pin.bottomCloud;
            continue;
        }
        const auto& pts = pin.bottomCloud;
        const int n = static_cast<int>(pts.size());

        // PCA法向量一致性检查
        std::vector<bool> normalOk(n, false);
        for (int i = 0; i < n; ++i) {
            // 找邻域点(用简单距离搜索替代KdTree)
            std::vector<Eigen::Vector3d> neighbors;
            for (int j = 0; j < n; ++j) {
                if ((pts[i] - pts[j]).norm() < CLUSTER_DIST)
                    neighbors.push_back(pts[j]);
            }
            if (static_cast<int>(neighbors.size()) < 3) continue;

            // 局部PCA
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (const auto& p : neighbors) mean += p;
            mean /= neighbors.size();

            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (const auto& p : neighbors) {
                Eigen::Vector3d d = p - mean;
                cov += d * d.transpose();
            }
            cov /= neighbors.size();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
            Eigen::Vector3d localNormal =
                solver.eigenvectors().col(0).normalized();

            double cosAngle = std::abs(localNormal.dot(bodyNormal));
            double angle = std::acos(std::clamp(cosAngle, 0.0, 1.0))
                           * 180.0 / M_PI;
            normalOk[i] = (angle < THETA_THR);
        }

        // 保留法向量一致的点
        std::vector<Eigen::Vector3d> filtered;
        for (int i = 0; i < n; ++i)
            if (normalOk[i]) filtered.push_back(pts[i]);

        // 简单连通性：保留最大簇
        if (filtered.size() < 3) {
            pin.refinedCloud = filtered.empty() ? pin.bottomCloud : filtered;
            continue;
        }

        // 标记簇
        std::vector<int> clusterIds(filtered.size(), -1);
        int clusterId = 0;
        for (size_t i = 0; i < filtered.size(); ++i) {
            if (clusterIds[i] >= 0) continue;
            clusterIds[i] = clusterId;
            std::vector<size_t> queue = {i};
            while (!queue.empty()) {
                size_t cur = queue.back(); queue.pop_back();
                for (size_t j = 0; j < filtered.size(); ++j) {
                    if (clusterIds[j] >= 0) continue;
                    if ((filtered[cur] - filtered[j]).norm() < CLUSTER_DIST) {
                        clusterIds[j] = clusterId;
                        queue.push_back(j);
                    }
                }
            }
            ++clusterId;
        }

        // 找最大簇
        std::vector<int> clusterSizes(clusterId, 0);
        for (int id : clusterIds) if (id >= 0) ++clusterSizes[id];
        int bestCluster = static_cast<int>(std::distance(
            clusterSizes.begin(),
            std::max_element(clusterSizes.begin(), clusterSizes.end())));

        pin.refinedCloud.clear();
        for (size_t i = 0; i < filtered.size(); ++i)
            if (clusterIds[i] == bestCluster)
                pin.refinedCloud.push_back(filtered[i]);
    }
}

// 功能：JEDEC座落平面共面度计算
QFPCoplanarityResult BGAMeasurePipeline::calcQFPCoplanarity(
    std::vector<PinBottomCloud>& pins)
{
    if (pins.size() < 3)
        throw tp::MeasureException("calcQFPCoplanarity: need >= 3 pins");

    constexpr int MAX_TUKEY_ITER = 5;
    constexpr double KAPPA = 1.5;

    // Step1: 逐引脚PCA平面拟合 + Tukey剔除
    for (auto& pin : pins) {
        auto& cloud = pin.refinedCloud.empty()
                          ? pin.bottomCloud : pin.refinedCloud;
        if (cloud.size() < 3) continue;

        for (int iter = 0; iter < MAX_TUKEY_ITER; ++iter) {
            const int n = static_cast<int>(cloud.size());
            if (n < 3) break;
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (const auto& p : cloud) mean += p;
            mean /= n;

            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (const auto& p : cloud) {
                Eigen::Vector3d d = p - mean;
                cov += d * d.transpose();
            }
            cov /= n;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
            Eigen::Vector3d normal =
                solver.eigenvectors().col(0).normalized();
            double d = -normal.dot(mean);
            pin.localPlane = Eigen::Vector4d(
                normal(0), normal(1), normal(2), d);

            // Tukey IQR 剔除
            std::vector<double> dists(n);
            for (int i = 0; i < n; ++i)
                dists[i] = normal.dot(cloud[i]) + d;

            auto sorted = dists;
            std::sort(sorted.begin(), sorted.end());
            double q1 = sorted[n / 4];
            double q3 = sorted[3 * n / 4];
            double iqr = q3 - q1;
            double lo = q1 - KAPPA * iqr;
            double hi = q3 + KAPPA * iqr;

            std::vector<Eigen::Vector3d> inliers;
            for (int i = 0; i < n; ++i)
                if (dists[i] >= lo && dists[i] <= hi)
                    inliers.push_back(cloud[i]);
            if (inliers.size() == cloud.size() || inliers.size() < 3) break;
            cloud = std::move(inliers);
        }
    }

    // Step2: 找全局底部方向 → 合并点云PCA
    std::vector<Eigen::Vector3d> allPts;
    for (const auto& pin : pins) {
        const auto& cloud = pin.refinedCloud.empty()
                                ? pin.bottomCloud : pin.refinedCloud;
        allPts.insert(allPts.end(), cloud.begin(), cloud.end());
    }
    Eigen::Vector3d globalMean = Eigen::Vector3d::Zero();
    for (const auto& p : allPts) globalMean += p;
    globalMean /= allPts.size();

    Eigen::Matrix3d globalCov = Eigen::Matrix3d::Zero();
    for (const auto& p : allPts) {
        Eigen::Vector3d d = p - globalMean;
        globalCov += d * d.transpose();
    }
    globalCov /= allPts.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> gSolver(globalCov);
    Eigen::Vector3d globalNormal =
        gSolver.eigenvectors().col(0).normalized();
    // 确保法向量指向底部(投影最小值一侧)
    double projMin = std::numeric_limits<double>::max();
    for (const auto& p : allPts)
        projMin = std::min(projMin, globalNormal.dot(p));
    if (globalNormal.dot(globalMean) - projMin < 0)
        globalNormal = -globalNormal;

    // Step3: 每引脚的floor point(最底部投影点)
    struct FloorInfo {
        int idx;
        double proj;
        Eigen::Vector3d point;
    };
    std::vector<FloorInfo> floors;
    for (size_t i = 0; i < pins.size(); ++i) {
        const auto& cloud = pins[i].refinedCloud.empty()
                                ? pins[i].bottomCloud
                                : pins[i].refinedCloud;
        if (cloud.empty()) continue;
        double minP = std::numeric_limits<double>::max();
        Eigen::Vector3d best;
        for (const auto& p : cloud) {
            double proj = globalNormal.dot(p);
            if (proj < minP) { minP = proj; best = p; }
        }
        floors.push_back({static_cast<int>(i), minP, best});
    }
    std::sort(floors.begin(), floors.end(),
              [](const FloorInfo& a, const FloorInfo& b) {
                  return a.proj < b.proj;
              });

    // 3个最底部且非共线点定义座落平面
    QFPCoplanarityResult result;
    Eigen::Vector3d sp0 = floors[0].point;
    Eigen::Vector3d sp1 = floors[1].point;
    Eigen::Vector3d seatNormal;
    int thirdIdx = 2;
    for (; thirdIdx < static_cast<int>(floors.size()); ++thirdIdx) {
        Eigen::Vector3d cross =
            (sp1 - sp0).cross(floors[thirdIdx].point - sp0);
        if (cross.norm() > 1e-8) {
            seatNormal = cross.normalized();
            break;
        }
    }
    if (thirdIdx >= static_cast<int>(floors.size()))
        seatNormal = globalNormal;

    double seatD = -seatNormal.dot(sp0);
    // 确保座落面法向量一致方向
    if (seatNormal.dot(globalNormal) < 0) {
        seatNormal = -seatNormal;
        seatD = -seatD;
    }
    result.seatingPlane =
        Eigen::Vector4d(seatNormal(0), seatNormal(1), seatNormal(2), seatD);
    result.seatingPinIds = {
        pins[floors[0].idx].pinId,
        pins[floors[1].idx].pinId,
        pins[floors[thirdIdx].idx].pinId};

    // Step4: 共面度 = 每引脚最低点到座落面距离的最大值
    double maxDev = 0;
    for (auto& pin : pins) {
        const auto& cloud = pin.refinedCloud.empty()
                                ? pin.bottomCloud : pin.refinedCloud;
        double minDist = std::numeric_limits<double>::max();
        for (const auto& p : cloud) {
            double dist = seatNormal.dot(p) + seatD;
            minDist = std::min(minDist, dist);
        }
        pin.deviation = minDist;
        maxDev = std::max(maxDev, minDist);

        // 翘曲角
        Eigen::Vector3d pn(pin.localPlane(0), pin.localPlane(1),
                           pin.localPlane(2));
        double cosA = std::abs(pn.dot(seatNormal));
        pin.warpAngle = std::acos(std::clamp(cosA, 0.0, 1.0))
                        * 180.0 / M_PI;
    }

    result.coplanarityValue = maxDev;
    result.maxDeviation = maxDev;
    result.pins = pins;
    return result;
}

} // namespace tp
