#define _USE_MATH_DEFINES
#include "measurement/BGADetector.h"
#include "common/Exceptions.h"
#include "common/Logger.h"
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace tp {

// ========== PCA 平面拟合 ==========
Eigen::Vector3d BGADetector::fitPCAPlane(
    const std::vector<Eigen::Vector3d>& pts,
    Eigen::Vector3d& centroid, double& d)
{
    const int n = static_cast<int>(pts.size());
    centroid = Eigen::Vector3d::Zero();
    for (const auto& p : pts) centroid += p;
    centroid /= n;

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pts) {
        Eigen::Vector3d diff = p - centroid;
        cov += diff * diff.transpose();
    }
    cov /= n;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    normal.normalize();
    if (normal(2) > 0) normal = -normal;
    d = -normal.dot(centroid);
    return normal;
}

// ========== 焊球掩膜检测 ==========
DetectionMasks BGADetector::detectBallMask(const cv::Mat& gray, const BGAConfig& cfg)
{
    auto& log = Logger::instance();
    log.info("detectBallMask: input size=" + std::to_string(gray.cols) + "x" + std::to_string(gray.rows));

    // 1. 芯片区域粗定位
    cv::Mat bwChip;
    cv::threshold(gray, bwChip, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Mat se50 = cv::getStructuringElement(cv::MORPH_RECT, {50, 50});
    cv::Mat eroded, morphed;
    cv::erode(bwChip, eroded, se50);
    cv::dilate(eroded, morphed, se50);
    cv::Mat inverted;
    cv::bitwise_not(morphed, inverted);

    cv::Mat labels, stats, centroids;
    int ncc = cv::connectedComponentsWithStats(inverted, labels, stats, centroids);
    int maxArea = 0, maxIdx = -1;
    for (int i = 1; i < ncc; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > maxArea) { maxArea = area; maxIdx = i; }
    }
    if (maxIdx < 0) return {cv::Mat::zeros(gray.size(), CV_8U), cv::Mat::zeros(gray.size(), CV_8U)};
    cv::Mat chipMask = cv::Mat::zeros(gray.size(), CV_8U);
    chipMask.setTo(255, (labels == maxIdx));

    // 2. 基板区域分割 — 仅对芯片区域像素计算 Otsu 阈值（与 MATLAB graythresh 一致）
    std::vector<uchar> roiPixels;
    roiPixels.reserve(maxArea);
    for (int r = 0; r < gray.rows; ++r)
        for (int c = 0; c < gray.cols; ++c)
            if (chipMask.at<uchar>(r, c))
                roiPixels.push_back(gray.at<uchar>(r, c));
    cv::Mat roiMat(roiPixels);
    cv::Mat roiDummy;
    double otsuThresh = cv::threshold(roiMat, roiDummy, 0, 255,
                                       cv::THRESH_BINARY | cv::THRESH_OTSU);
    log.info("detectBallMask: chipArea=" + std::to_string(maxArea)
        + " roiPixels=" + std::to_string(roiPixels.size())
        + " otsuThresh=" + std::to_string(otsuThresh));

    cv::Mat bw;
    cv::threshold(gray, bw, otsuThresh, 255, cv::THRESH_BINARY);
    cv::Mat bwSub;
    cv::bitwise_not(bw, bwSub);
    bwSub.setTo(0, ~chipMask);

    ncc = cv::connectedComponentsWithStats(bwSub, labels, stats, centroids);
    maxArea = 0; maxIdx = -1;
    for (int i = 1; i < ncc; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > maxArea) { maxArea = area; maxIdx = i; }
    }
    if (maxIdx < 0) return {cv::Mat::zeros(gray.size(), CV_8U), chipMask};
    cv::Mat substrateMask = cv::Mat::zeros(gray.size(), CV_8U);
    substrateMask.setTo(255, (labels == maxIdx));

    // 3. 形态学闭运算（自适应半径, 与MATLAB一致: r = ceil(min(h,w)/1000 * 2)）
    double rho = std::min(gray.rows, gray.cols) / 1000.0;
    int r = static_cast<int>(std::ceil(rho * 2.0));
    r = std::max(r, 3);
    cv::Mat seClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, {2*r+1, 2*r+1});
    cv::Mat substrateClosed;
    cv::morphologyEx(substrateMask, substrateClosed, cv::MORPH_CLOSE, seClose);

    // 4. 焊球候选
    cv::Mat ballCandidates;
    cv::bitwise_not(substrateClosed, ballCandidates);
    ballCandidates.setTo(0, ~chipMask);
    cv::threshold(ballCandidates, ballCandidates, 127, 255, cv::THRESH_BINARY);

    // 5. 圆形度和面积筛选
    ncc = cv::connectedComponentsWithStats(ballCandidates, labels, stats, centroids);
    log.info("detectBallMask: ballCandidates components=" + std::to_string(ncc - 1));
    cv::Mat ballMask = cv::Mat::zeros(gray.size(), CV_8U);

    // bwperim 等效：形态学边界（MATLAB: bwperim + bwdist）
    cv::Mat eroded1px;
    cv::erode(substrateClosed, eroded1px, cv::Mat());
    cv::Mat substrateBoundary;
    cv::subtract(substrateClosed, eroded1px, substrateBoundary);
    cv::Mat distMap;
    cv::distanceTransform(255 - substrateBoundary, distMap, cv::DIST_L2, 3);

    for (int i = 1; i < ncc; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < cfg.areaMin || area > cfg.areaMax) continue;

        cv::Mat comp = (labels == i);
        comp.convertTo(comp, CV_8U, 255);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(comp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        if (contours.empty()) continue;
        double perimeter = cv::arcLength(contours[0], true);
        if (perimeter <= 0) continue;
        double circ = 4.0 * M_PI * area / (perimeter * perimeter);
        if (circ < cfg.circularityThresh) continue;

        int cx = static_cast<int>(centroids.at<double>(i, 0));
        int cy = static_cast<int>(centroids.at<double>(i, 1));
        cx = std::clamp(cx, 0, gray.cols - 1);
        cy = std::clamp(cy, 0, gray.rows - 1);
        if (distMap.at<float>(cy, cx) < cfg.edgeDistThresh) continue;

        ballMask.setTo(255, (labels == i));
    }
    int finalCount = cv::countNonZero(ballMask);
    log.info("detectBallMask: final ballMask nonzero=" + std::to_string(finalCount));
    return {ballMask, chipMask};
}

// ========== allDealTwoplane 全流程 ==========
std::vector<BallResult> BGADetector::runAllDealTwoplane(
    const cv::Mat& bgaImage,
    const cv::Mat& X, const cv::Mat& Y, const cv::Mat& Z,
    const BGAConfig& config)
{
    auto& log = Logger::instance();
    log.info("allDealTwoplane: start");

    cv::Mat gray;
    if (bgaImage.channels() == 3)
        cv::cvtColor(bgaImage, gray, cv::COLOR_BGR2GRAY);
    else
        gray = bgaImage.clone();

    // Z翻转
    cv::Mat Zf = Z.clone();
    if (config.zFlip > 0) Zf = config.zFlip - Zf;

    // 第一部分：焊球检测
    auto masks = detectBallMask(gray, config);
    cv::Mat ballMask = masks.ballMask;
    cv::Mat chipMask = masks.chipMask;

    // 焊球掩膜腐蚀/膨胀（与MATLAB一致: erode disk(2), dilate disk(5)）
    cv::Mat seErode = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
    cv::Mat seDilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, {11, 11});
    cv::Mat ballMaskEroded, ballMaskDilated;
    cv::erode(ballMask, ballMaskEroded, seErode);
    cv::dilate(ballMask, ballMaskDilated, seDilate);

    // 芯片掩膜腐蚀（与MATLAB一致: erode disk(2)）
    cv::Mat chipMaskEroded;
    cv::erode(chipMask, chipMaskEroded, seErode);

    cv::Mat labels, stats, centroids;
    int nBalls = cv::connectedComponentsWithStats(ballMask, labels, stats, centroids) - 1;
    if (nBalls < 1)
        throw MeasureException("allDealTwoplane: no valid balls detected");

    log.info("allDealTwoplane: detected " + std::to_string(nBalls) + " balls");

    struct BallInfo { double xc, yc, radius; int labelIdx; };
    std::vector<BallInfo> balls;
    for (int i = 1; i <= nBalls; ++i) {
        BallInfo bi;
        bi.xc = centroids.at<double>(i, 0);
        bi.yc = centroids.at<double>(i, 1);
        bi.radius = std::sqrt(stats.at<int>(i, cv::CC_STAT_AREA) / M_PI);
        bi.labelIdx = i;
        balls.push_back(bi);
    }

    // 第二部分：基板PCA平面（MATLAB §2.2: substrateMask = I_masked & ~hanqiuMask_dilated & validDataMask）
    cv::Mat validZ = cv::Mat::zeros(Zf.size(), CV_8U);
    for (int r2 = 0; r2 < Zf.rows; ++r2)
        for (int c2 = 0; c2 < Zf.cols; ++c2) {
            double z = Zf.at<double>(r2, c2);
            if (z != 0 && std::isfinite(z)) validZ.at<uchar>(r2, c2) = 255;
        }
    cv::Mat notBallDilated;
    cv::bitwise_not(ballMaskDilated, notBallDilated);
    cv::Mat substSample;
    cv::bitwise_and(chipMaskEroded, notBallDilated, substSample);
    cv::bitwise_and(substSample, validZ, substSample);

    std::vector<Eigen::Vector3d> subPts;
    for (int r = 0; r < Zf.rows; ++r) {
        for (int c = 0; c < Zf.cols; ++c) {
            if (substSample.at<uchar>(r, c) == 0) continue;
            double z = Zf.at<double>(r, c);
            if (z == 0 || !std::isfinite(z)) continue;
            double x = X.at<double>(r, c);
            double y = Y.at<double>(r, c);
            if (!std::isfinite(x) || !std::isfinite(y)) continue;
            subPts.emplace_back(x, y, z);
        }
    }
    if (subPts.size() < 30)
        throw MeasureException("allDealTwoplane: too few substrate points");

    Eigen::Vector3d subCentroid;
    substrateNormal_ = fitPCAPlane(subPts, subCentroid, substrateD_);
    log.info("allDealTwoplane: substrate normal=[" +
        std::to_string(substrateNormal_(0)) + "," +
        std::to_string(substrateNormal_(1)) + "," +
        std::to_string(substrateNormal_(2)) + "]");

    // 第三部分：焊球高度测量
    cv::Mat labelsEr, statsEr, centroidsEr;
    int nEr = cv::connectedComponentsWithStats(ballMaskEroded, labelsEr, statsEr, centroidsEr);

    std::vector<BallResult> results(balls.size());
    int successCount = 0;

    for (size_t bi = 0; bi < balls.size(); ++bi) {
        auto& br = results[bi];
        br.xc = balls[bi].xc;
        br.yc = balls[bi].yc;
        br.radius = balls[bi].radius;

        double minDist = 1e9;
        int bestLabel = -1;
        for (int j = 1; j < nEr; ++j) {
            double cx = centroidsEr.at<double>(j, 0);
            double cy = centroidsEr.at<double>(j, 1);
            double d = std::hypot(cx - br.xc, cy - br.yc);
            if (d < minDist) { minDist = d; bestLabel = j; }
        }
        if (bestLabel < 0) continue;

        std::vector<Eigen::Vector3d> ballPts;
        for (int r = 0; r < Zf.rows; ++r) {
            for (int c = 0; c < Zf.cols; ++c) {
                if (labelsEr.at<int>(r, c) != bestLabel) continue;
                double z = Zf.at<double>(r, c);
                if (z == 0 || !std::isfinite(z)) continue;
                double x = X.at<double>(r, c);
                double y = Y.at<double>(r, c);
                if (!std::isfinite(x) || !std::isfinite(y)) continue;
                ballPts.emplace_back(x, y, z);
            }
        }
        if (static_cast<int>(ballPts.size()) < config.minBallPoints) continue;

        // 单焊球PCA平面（MATLAB §2.4: 对每个焊球点云做PCA取法向量）
        Eigen::Vector3d ballCentPCA;
        double ballD;
        Eigen::Vector3d ballNormal = fitPCAPlane(ballPts, ballCentPCA, ballD);
        // 对齐方向：令球法向与基板法向同向
        if (ballNormal.dot(substrateNormal_) < 0)
            ballNormal = -ballNormal;

        // 高度 = 焊球质心到基板平面的有符号距离
        double height = std::abs(substrateNormal_.dot(ballCentPCA) + substrateD_);

        br.x3d = ballCentPCA(0);
        br.y3d = ballCentPCA(1);
        br.z3d = ballCentPCA(2);
        br.height = height;
        br.nx = ballNormal(0);
        br.ny = ballNormal(1);
        br.nz = ballNormal(2);
        br.success = true;
        ++successCount;
    }

    log.info("allDealTwoplane: done success=" + std::to_string(successCount));

    // ========== 芯片旋转矫正与焊球排序（MATLAB §1.10）==========
    // Canny 边缘检测芯片轮廓
    cv::Mat chipEdge;
    cv::Canny(chipMask, chipEdge, 50, 150);
    std::vector<cv::Point> edgePoints;
    for (int er = 0; er < chipEdge.rows; ++er)
        for (int ec = 0; ec < chipEdge.cols; ++ec)
            if (chipEdge.at<uchar>(er, ec))
                edgePoints.emplace_back(ec, er);

    double rotAngle = 0;
    if (edgePoints.size() >= 100) {
        // PCA on edge points to find principal direction
        double meanEX = 0, meanEY = 0;
        for (const auto& ep : edgePoints) {
            meanEX += ep.x;
            meanEY += ep.y;
        }
        meanEX /= edgePoints.size();
        meanEY /= edgePoints.size();

        cv::Mat edgeMat(static_cast<int>(edgePoints.size()), 2, CV_64F);
        for (size_t i = 0; i < edgePoints.size(); ++i) {
            edgeMat.at<double>(static_cast<int>(i), 0) = edgePoints[i].x - meanEX;
            edgeMat.at<double>(static_cast<int>(i), 1) = edgePoints[i].y - meanEY;
        }
        cv::PCA pca2d(edgeMat, cv::Mat(), cv::PCA::DATA_AS_ROW);
        double dx = pca2d.eigenvectors.at<double>(0, 0);
        double dy = pca2d.eigenvectors.at<double>(0, 1);
        double mainAngleDeg = std::atan2(dy, dx) * 180.0 / M_PI;
        rotAngle = -mainAngleDeg;
    }
    log.info("allDealTwoplane: rotation angle=" + std::to_string(rotAngle) + " deg");

    // 旋转变换：将所有焊球质心旋转到对齐坐标系
    double theta = rotAngle * M_PI / 180.0;
    double cosT = std::cos(theta), sinT = std::sin(theta);
    double imgCx = gray.cols / 2.0, imgCy = gray.rows / 2.0;
    std::vector<double> rotatedX(results.size()), rotatedY(results.size());
    for (size_t i = 0; i < results.size(); ++i) {
        double dxx = results[i].xc - imgCx;
        double dyy = results[i].yc - imgCy;
        rotatedX[i] = cosT * dxx - sinT * dyy + imgCx;
        rotatedY[i] = sinT * dxx + cosT * dyy + imgCy;
    }

    // 按旋转后Y坐标排序（行检测），使用IQR自适应阈值（MATLAB §1.10）
    std::vector<size_t> sortIdx(results.size());
    std::iota(sortIdx.begin(), sortIdx.end(), 0);
    std::sort(sortIdx.begin(), sortIdx.end(),
        [&](size_t a, size_t b) { return rotatedY[a] < rotatedY[b]; });

    std::vector<double> yDiffs;
    for (size_t i = 1; i < sortIdx.size(); ++i)
        yDiffs.push_back(rotatedY[sortIdx[i]] - rotatedY[sortIdx[i - 1]]);

    double rowThreshold = 50.0;
    if (yDiffs.size() >= 3) {
        auto sorted = yDiffs;
        std::sort(sorted.begin(), sorted.end());
        double q1 = sorted[sorted.size() / 4];
        double q3 = sorted[3 * sorted.size() / 4];
        double iqr = q3 - q1;
        if (iqr > 0) {
            rowThreshold = q3 + 1.5 * iqr;
        } else {
            // MATLAB fallback: mean + 2*std
            double mean = 0;
            for (double v : sorted) mean += v;
            mean /= sorted.size();
            double sumSq = 0;
            for (double v : sorted) sumSq += (v - mean) * (v - mean);
            double stdDev = std::sqrt(sumSq / sorted.size());
            rowThreshold = mean + 2.0 * stdDev;
        }
        // MATLAB: row_threshold = max(row_threshold, median * 1.5)
        rowThreshold = std::max(rowThreshold, sorted[sorted.size() / 2] * 1.5);
        log.info("allDealTwoplane: Q1=" + std::to_string(q1) +
                 " Q3=" + std::to_string(q3) + " IQR=" + std::to_string(iqr) +
                 " rowThreshold=" + std::to_string(rowThreshold));
    }

    int currentRow = 1;
    results[sortIdx[0]].row = currentRow;
    for (size_t i = 1; i < sortIdx.size(); ++i) {
        if (yDiffs[i - 1] > rowThreshold) {
            ++currentRow;
            log.info("allDealTwoplane: new row at index " + std::to_string(i) +
                     " yDiff=" + std::to_string(yDiffs[i - 1]));
        }
        results[sortIdx[i]].row = currentRow;
    }

    // 每行内按旋转后X坐标排序，分配全局编号
    int globalOrder = 1;
    for (int row = 1; row <= currentRow; ++row) {
        std::vector<size_t> rowBalls;
        for (size_t i = 0; i < results.size(); ++i)
            if (results[i].row == row) rowBalls.push_back(i);
        std::sort(rowBalls.begin(), rowBalls.end(),
            [&](size_t a, size_t b) { return rotatedX[a] < rotatedX[b]; });
        for (auto idx : rowBalls)
            results[idx].order = globalOrder++;
    }

    // 输出每行焊球数量
    std::string rowCountStr;
    for (int row = 1; row <= currentRow; ++row) {
        int cnt = 0;
        for (const auto& b : results)
            if (b.row == row) ++cnt;
        rowCountStr += std::to_string(cnt) + " ";
    }
    log.info("allDealTwoplane: " + std::to_string(currentRow) +
             " rows, per-row counts: " + rowCountStr);

    return results;
}

// ========== 兼容旧接口 ==========
std::vector<cv::Vec3f> BGADetector::detect2DBalls(
    const cv::Mat& brightImage, const cv::Mat& /*mask*/)
{
    cv::Mat gray;
    if (brightImage.channels() == 3)
        cv::cvtColor(brightImage, gray, cv::COLOR_BGR2GRAY);
    else gray = brightImage;

    cv::Mat ballMask = detectBallMask(gray, {}).ballMask;
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(ballMask, labels, stats, centroids);
    std::vector<cv::Vec3f> result;
    for (int i = 1; i < n; ++i) {
        float cx = static_cast<float>(centroids.at<double>(i, 0));
        float cy = static_cast<float>(centroids.at<double>(i, 1));
        float r = static_cast<float>(std::sqrt(stats.at<int>(i, cv::CC_STAT_AREA) / M_PI));
        result.push_back({cx, cy, r});
    }
    return result;
}

std::vector<Eigen::Vector3d> BGADetector::localize3DBalls(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const std::vector<cv::Vec3f>& balls2D,
    const CalibData& calib)
{
    if (!cloud || cloud->empty())
        throw MeasureException("localize3DBalls: empty cloud");
    if (balls2D.empty())
        throw MeasureException("localize3DBalls: no 2D balls");

    constexpr int MIN_PTS = 10;
    bool hasXY = !calib.X.empty() && !calib.Y.empty();
    std::vector<Eigen::Vector3d> verts;

    for (const auto& b : balls2D) {
        int cx = static_cast<int>(std::round(b[0]));
        int cy = static_cast<int>(std::round(b[1]));
        double wx, wy, wr;
        if (hasXY && cy >= 0 && cy < calib.X.rows && cx >= 0 && cx < calib.X.cols) {
            wx = calib.X.at<double>(cy, cx);
            wy = calib.Y.at<double>(cy, cx);
            int ex = std::min(cx + static_cast<int>(b[2]), calib.X.cols - 1);
            wr = std::hypot(calib.X.at<double>(cy, ex) - wx,
                            calib.Y.at<double>(cy, ex) - wy);
            if (wr < 0.01) wr = 2.0;
        } else { wx = b[0]; wy = b[1]; wr = b[2]; }

        double sr = wr * 2.0;
        std::vector<Eigen::Vector3d> pts;
        for (const auto& pt : cloud->points) {
            if (!std::isfinite(pt.z)) continue;
            double dx = pt.x - wx, dy = pt.y - wy;
            if (dx * dx + dy * dy <= sr * sr) pts.emplace_back(pt.x, pt.y, pt.z);
        }
        if (static_cast<int>(pts.size()) < MIN_PTS) continue;
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const auto& p : pts) c += p;
        c /= pts.size();
        verts.push_back(c);
    }
    return verts;
}

double BGADetector::calcCoplanarity(const std::vector<Eigen::Vector3d>& points)
{
    if (points.size() < 3)
        throw MeasureException("calcCoplanarity: need >= 3 points");

    const int n = static_cast<int>(points.size());
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) centroid += p;
    centroid /= n;

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= n;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    normal.normalize();
    double dd = -normal.dot(centroid);

    std::vector<double> dists(n);
    for (int i = 0; i < n; ++i) dists[i] = normal.dot(points[i]) + dd;

    std::vector<int> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    std::partial_sort(idx.begin(), idx.begin() + 3, idx.end(),
        [&](int a, int b) { return dists[a] < dists[b]; });

    Eigen::Vector3d seatN = (points[idx[1]] - points[idx[0]]).cross(
                             points[idx[2]] - points[idx[0]]);
    if (seatN.norm() < 1e-12) return 0.0;
    seatN.normalize();
    double seatD = -seatN.dot(points[idx[0]]);

    double maxDev = 0;
    for (const auto& p : points)
        maxDev = std::max(maxDev, std::abs(seatN.dot(p) + seatD));
    return maxDev;
}

} // namespace tp
