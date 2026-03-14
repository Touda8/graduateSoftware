#define _USE_MATH_DEFINES
#include "BGADetector.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace tp {

// 功能：最小二乘圆拟合(代数法)，输入：2D点集，返回：(cx,cy,r)
cv::Vec3d BGADetector::fitCircleIterative(
    const std::vector<cv::Point2d>& pts, int maxIter, double beta)
{
    if (pts.size() < 5) {
        throw tp::MeasureException("fitCircleIterative: need >= 5 points");
    }
    std::vector<cv::Point2d> current = pts;

    double xc = 0, yc = 0, radius = 0;
    for (int iter = 0; iter < maxIter; ++iter) {
        const int n = static_cast<int>(current.size());
        if (n < 5) break;

        // 代数圆拟合: (x-xc)²+(y-yc)²=r² → ax+by+c = -(x²+y²)
        Eigen::MatrixXd A(n, 3);
        Eigen::VectorXd b(n);
        for (int i = 0; i < n; ++i) {
            A(i, 0) = current[i].x;
            A(i, 1) = current[i].y;
            A(i, 2) = 1.0;
            b(i) = -(current[i].x * current[i].x +
                      current[i].y * current[i].y);
        }
        Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
        xc = -sol(0) / 2.0;
        yc = -sol(1) / 2.0;
        radius = std::sqrt(xc * xc + yc * yc - sol(2));

        // 径向偏差
        std::vector<double> errors(n);
        for (int i = 0; i < n; ++i) {
            double d = std::hypot(current[i].x - xc, current[i].y - yc);
            errors[i] = std::abs(d - radius);
        }
        double sigma = 0;
        for (double e : errors) sigma += e * e;
        sigma = std::sqrt(sigma / n);
        double thresh = beta * sigma;

        // 剔除离群点
        std::vector<cv::Point2d> inliers;
        for (int i = 0; i < n; ++i) {
            if (errors[i] <= thresh) inliers.push_back(current[i]);
        }
        if (inliers.size() < 5 ||
            inliers.size() == current.size()) break;
        current = std::move(inliers);
    }
    return cv::Vec3d(xc, yc, radius);
}

// 功能：BGA焊球二维检测，输入：灰度图+掩膜，返回：(cx,cy,r)列表
std::vector<cv::Vec3f> BGADetector::detect2DBalls(
    const cv::Mat& brightImage, const cv::Mat& mask)
{
    if (brightImage.empty()) {
        throw tp::ImageLoadException("detect2DBalls: brightImage is empty");
    }

    constexpr double CIRC_THRESH = 0.82;
    constexpr int AREA_MIN = 300;
    constexpr int AREA_MAX = 4000;
    constexpr int MAX_CIRCLE_ITER = 10;
    constexpr double CIRCLE_BETA = 2.0;

    cv::Mat gray;
    if (brightImage.channels() == 3)
        cv::cvtColor(brightImage, gray, cv::COLOR_BGR2GRAY);
    else
        gray = brightImage.clone();

    // 在掩膜内进行阈值分割
    cv::Mat roi = gray.clone();
    if (!mask.empty()) roi.setTo(0, ~mask);

    cv::Mat threshTmp;
    double level = cv::threshold(roi, threshTmp, 0, 255,
                                 cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Mat binary;
    cv::threshold(roi, binary, level, 255, cv::THRESH_BINARY_INV);
    if (!mask.empty()) cv::bitwise_and(binary, mask, binary);

    // 形态学闭运算
    auto se = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, se);

    // 反转获取焊球候选 (高灰度 = 焊球)
    cv::Mat invBinary;
    cv::bitwise_not(binary, invBinary);
    if (!mask.empty()) cv::bitwise_and(invBinary, mask, invBinary);

    // 连通域分析
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(
        invBinary, labels, stats, centroids, 8, CV_32S);

    std::vector<cv::Vec3f> result;
    for (int i = 1; i < nLabels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < AREA_MIN || area > AREA_MAX) continue;

        // 计算周长和圆形度
        cv::Mat compMask = (labels == i);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(compMask, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_NONE);
        if (contours.empty()) continue;
        double perimeter = cv::arcLength(contours[0], true);
        if (perimeter <= 0) continue;
        double circularity = 4.0 * M_PI * area / (perimeter * perimeter);
        if (circularity < CIRC_THRESH) continue;

        // 提取边缘点做迭代圆拟合
        std::vector<cv::Point2d> edgePts;
        cv::Mat edges;
        cv::Canny(compMask, edges, 50, 150);
        for (int r = 0; r < edges.rows; ++r)
            for (int c = 0; c < edges.cols; ++c)
                if (edges.at<uchar>(r, c))
                    edgePts.emplace_back(c, r);
        if (edgePts.size() < 5) {
            // 回退到质心+面积估计
            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            float r = static_cast<float>(std::sqrt(area / M_PI));
            result.emplace_back(static_cast<float>(cx),
                                static_cast<float>(cy), r);
            continue;
        }
        cv::Vec3d circleRes = fitCircleIterative(
            edgePts, MAX_CIRCLE_ITER, CIRCLE_BETA);
        result.emplace_back(static_cast<float>(circleRes[0]),
                            static_cast<float>(circleRes[1]),
                            static_cast<float>(circleRes[2]));
    }

    // E07: 焊球检测结果为空时抛出异常
    if (result.empty()) {
        throw tp::MeasureException("detect2DBalls: no balls detected");
    }
    return result;
}

// 功能：二阶曲面拟合 Z=ax²+by²+cxy+dx+ey+f
std::array<double, 6> BGADetector::fitQuadSurface(
    const std::vector<Eigen::Vector3d>& pts)
{
    const int n = static_cast<int>(pts.size());
    if (n < 6) throw tp::MeasureException("fitQuadSurface: need >= 6 pts");

    Eigen::MatrixXd A(n, 6);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) {
        double x = pts[i](0), y = pts[i](1);
        A(i, 0) = x * x;
        A(i, 1) = y * y;
        A(i, 2) = x * y;
        A(i, 3) = x;
        A(i, 4) = y;
        A(i, 5) = 1.0;
        b(i) = pts[i](2);
    }
    Eigen::VectorXd sol = A.colPivHouseholderQr().solve(b);
    return {sol(0), sol(1), sol(2), sol(3), sol(4), sol(5)};
}

// 功能：BGA焊球3D顶点定位
std::vector<Eigen::Vector3d> BGADetector::localize3DBalls(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const std::vector<cv::Vec3f>& balls2D,
    const CalibData& calib)
{
    if (!cloud || cloud->empty()) {
        throw tp::MeasureException("localize3DBalls: empty cloud");
    }
    if (balls2D.empty()) {
        throw tp::MeasureException("localize3DBalls: no 2D balls");
    }

    constexpr int MIN_BALL_POINTS = 10;
    constexpr int MAX_SURFACE_ITER = 100;
    constexpr double OUTLIER_SIGMA = 2.5;

    // 将2D像素坐标球心转换为世界坐标，使用标定X/Y矩阵
    struct Ball3DSearch {
        double wx, wy, wRadius;
    };
    std::vector<Ball3DSearch> searchBalls;
    searchBalls.reserve(balls2D.size());

    bool hasCalibXY = !calib.X.empty() && !calib.Y.empty();

    for (const auto& ball : balls2D) {
        int cx = static_cast<int>(std::round(ball[0]));
        int cy = static_cast<int>(std::round(ball[1]));
        float radius = ball[2];
        Ball3DSearch bs{};

        if (hasCalibXY
            && cy >= 0 && cy < calib.X.rows
            && cx >= 0 && cx < calib.X.cols) {
            // 使用标定矩阵将像素中心映射到世界坐标
            bs.wx = calib.X.at<double>(cy, cx);
            bs.wy = calib.Y.at<double>(cy, cx);
            // 估算世界坐标半径：取球边缘一点计算距离
            int ex = std::min(cx + static_cast<int>(radius), calib.X.cols - 1);
            double edgeX = calib.X.at<double>(cy, ex);
            double edgeY = calib.Y.at<double>(cy, ex);
            bs.wRadius = std::sqrt((edgeX - bs.wx) * (edgeX - bs.wx)
                                 + (edgeY - bs.wy) * (edgeY - bs.wy));
            if (bs.wRadius < 0.01) bs.wRadius = 2.0; // fallback 2mm
        } else {
            // 无标定数据，直接使用像素坐标（原始行为，可能不精确）
            bs.wx = ball[0];
            bs.wy = ball[1];
            bs.wRadius = radius;
        }
        searchBalls.push_back(bs);
    }

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(balls2D.size());

    for (const auto& bs : searchBalls) {
        double searchR = bs.wRadius * 2.0;

        // 收集焊球区域内的3D点
        std::vector<Eigen::Vector3d> ballPts;
        for (const auto& pt : cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.z)) continue;
            double dx = pt.x - bs.wx;
            double dy = pt.y - bs.wy;
            if (dx * dx + dy * dy <= searchR * searchR) {
                ballPts.emplace_back(pt.x, pt.y, pt.z);
            }
        }
        if (static_cast<int>(ballPts.size()) < MIN_BALL_POINTS) continue;

        // 迭代曲面拟合
        auto current = ballPts;
        std::array<double, 6> params{};
        for (int iter = 0; iter < MAX_SURFACE_ITER; ++iter) {
            if (static_cast<int>(current.size()) < MIN_BALL_POINTS) break;
            params = fitQuadSurface(current);

            std::vector<double> residuals(current.size());
            for (size_t i = 0; i < current.size(); ++i) {
                double x = current[i](0), y = current[i](1);
                double zPred = params[0]*x*x + params[1]*y*y +
                               params[2]*x*y + params[3]*x +
                               params[4]*y + params[5];
                residuals[i] = std::abs(current[i](2) - zPred);
            }
            double sigma = 0;
            for (double r : residuals) sigma += r * r;
            sigma = std::sqrt(sigma / residuals.size());
            double thresh = OUTLIER_SIGMA * sigma;

            std::vector<Eigen::Vector3d> inliers;
            for (size_t i = 0; i < current.size(); ++i) {
                if (residuals[i] < thresh) inliers.push_back(current[i]);
            }
            if (inliers.size() == current.size() ||
                static_cast<int>(inliers.size()) < MIN_BALL_POINTS) break;
            current = std::move(inliers);
        }

        // 曲面极值点: dZ/dx=0, dZ/dy=0
        double a = params[0], b = params[1], c = params[2];
        double d = params[3], e = params[4], f = params[5];
        Eigen::Matrix2d coeff;
        coeff << 2*a, c, c, 2*b;
        Eigen::Vector2d rhs(-d, -e);
        if (std::abs(coeff.determinant()) > 1e-10) {
            Eigen::Vector2d xy = coeff.colPivHouseholderQr().solve(rhs);
            double zTop = a*xy(0)*xy(0) + b*xy(1)*xy(1) +
                          c*xy(0)*xy(1) + d*xy(0) + e*xy(1) + f;
            vertices.emplace_back(xy(0), xy(1), zTop);
        } else {
            // 回退：取Z最大点
            auto it = std::max_element(current.begin(), current.end(),
                [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                    return a(2) < b(2);
                });
            if (it != current.end()) vertices.push_back(*it);
        }
    }
    return vertices;
}

// 功能：共面度计算（JEDEC标准），输入：3D顶点坐标，返回：共面度(mm)
double BGADetector::calcCoplanarity(
    const std::vector<Eigen::Vector3d>& points)
{
    if (points.size() < 3) {
        throw tp::MeasureException("calcCoplanarity: need >= 3 points");
    }
    const int n = static_cast<int>(points.size());

    // PCA拟合参考平面
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) centroid += p;
    centroid /= n;

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d diff = p - centroid;
        cov += diff * diff.transpose();
    }
    cov /= n;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    normal.normalize();
    double d = -normal.dot(centroid);

    // 找座落平面: 3个最底部非共线点
    std::vector<std::pair<double, int>> projections(n);
    for (int i = 0; i < n; ++i) {
        projections[i] = {normal.dot(points[i]) + d, i};
    }
    std::sort(projections.begin(), projections.end());

    // 使用最低3点定义座落平面
    Eigen::Vector3d p0 = points[projections[0].second];
    Eigen::Vector3d p1 = points[projections[1].second];
    Eigen::Vector3d p2 = points[projections[2].second];

    Eigen::Vector3d seatNormal = (p1 - p0).cross(p2 - p0);
    if (seatNormal.norm() < 1e-12) {
        // 共线回退：使用PCA平面
        seatNormal = normal;
    } else {
        seatNormal.normalize();
    }
    double seatD = -seatNormal.dot(p0);

    // 计算每个点到座落平面的距离
    double maxDev = 0.0;
    for (const auto& p : points) {
        double dist = seatNormal.dot(p) + seatD;
        if (dist > maxDev) maxDev = dist;
    }
    return maxDev;
}

} // namespace tp
