/*
 * chart_bindgen.cpp - 图表绘制工具库
 * 使用 Windows GDI+ 绘制柱状图和折线图，输出 PNG
 * 编译: cl /EHsc /utf-8 /std:c++17 chart_bindgen.cpp /link gdiplus.lib
 */
#ifndef CHART_BINDGEN_HPP
#define CHART_BINDGEN_HPP

#define NOMINMAX
#include <windows.h>
#include <gdiplus.h>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <functional>

#pragma comment(lib, "gdiplus.lib")

// ============================================================
//  GDI+ 初始化 RAII
// ============================================================
struct GdiplusSession {
    ULONG_PTR token;
    GdiplusSession() {
        Gdiplus::GdiplusStartupInput in;
        Gdiplus::GdiplusStartup(&token, &in, nullptr);
    }
    ~GdiplusSession() { Gdiplus::GdiplusShutdown(token); }
};

// ============================================================
//  获取 PNG 编码器 CLSID
// ============================================================
static int GetEncoderClsid(const WCHAR* format, CLSID* pClsid) {
    UINT num = 0, size = 0;
    Gdiplus::GetImageEncodersSize(&num, &size);
    if (size == 0) return -1;
    auto pInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
    if (!pInfo) return -1;
    Gdiplus::GetImageEncoders(num, size, pInfo);
    for (UINT j = 0; j < num; ++j) {
        if (wcscmp(pInfo[j].MimeType, format) == 0) {
            *pClsid = pInfo[j].Clsid;
            free(pInfo);
            return j;
        }
    }
    free(pInfo);
    return -1;
}

// ============================================================
//  统计工具
// ============================================================
static double vec_mean(const std::vector<double>& v) {
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}
static double vec_std(const std::vector<double>& v) {
    double m = vec_mean(v);
    double s = 0;
    for (auto x : v) s += (x - m) * (x - m);
    return std::sqrt(s / v.size());
}
static double vec_min(const std::vector<double>& v) {
    return *std::min_element(v.begin(), v.end());
}
static double vec_max(const std::vector<double>& v) {
    return *std::max_element(v.begin(), v.end());
}

// ============================================================
//  CSV 读取
// ============================================================
struct CsvTable {
    std::vector<std::string> headers;
    std::vector<std::vector<double>> columns; // columns[col][row]
    int rows() const { return columns.empty() ? 0 : (int)columns[0].size(); }
    int cols() const { return (int)columns.size(); }
    std::vector<double> col(int c) const { return columns[c]; }
};

static CsvTable read_csv(const std::string& path) {
    CsvTable t;
    std::ifstream f(path);
    if (!f.is_open()) {
        fprintf(stderr, "ERROR: Cannot open CSV: %s\n", path.c_str());
        return t;
    }
    std::string line;
    // header
    if (std::getline(f, line)) {
        // 处理 UTF-8 BOM
        if (line.size() >= 3 && (unsigned char)line[0] == 0xEF
            && (unsigned char)line[1] == 0xBB && (unsigned char)line[2] == 0xBF) {
            line = line.substr(3);
        }
        std::istringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            t.headers.push_back(cell);
            t.columns.push_back({});
        }
    }
    // data
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string cell;
        int c = 0;
        while (std::getline(ss, cell, ',') && c < t.cols()) {
            try { t.columns[c].push_back(std::stod(cell)); }
            catch (...) { t.columns[c].push_back(0.0); }
            c++;
        }
    }
    return t;
}

// ============================================================
//  宽字符转换
// ============================================================
static std::wstring to_wstr(const std::string& s) {
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    std::wstring ws(n, 0);
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &ws[0], n);
    if (!ws.empty() && ws.back() == L'\0') ws.pop_back();
    return ws;
}

// ============================================================
//  轴刻度计算 (nice numbers)
// ============================================================
static double nice_num(double range, bool round_flag) {
    double exponent = std::floor(std::log10(range));
    double fraction = range / std::pow(10.0, exponent);
    double nice;
    if (round_flag) {
        if (fraction < 1.5) nice = 1; else if (fraction < 3) nice = 2;
        else if (fraction < 7) nice = 5; else nice = 10;
    } else {
        if (fraction <= 1) nice = 1; else if (fraction <= 2) nice = 2;
        else if (fraction <= 5) nice = 5; else nice = 10;
    }
    return nice * std::pow(10.0, exponent);
}

struct AxisTicks {
    double lo, hi, step;
    std::vector<double> ticks;
};

static AxisTicks calc_ticks(double dmin, double dmax, int nticks = 5) {
    AxisTicks ax;
    double range = nice_num(dmax - dmin, false);
    ax.step = nice_num(range / (nticks - 1), true);
    ax.lo = std::floor(dmin / ax.step) * ax.step;
    ax.hi = std::ceil(dmax / ax.step) * ax.step;
    for (double v = ax.lo; v <= ax.hi + ax.step * 0.001; v += ax.step)
        ax.ticks.push_back(v);
    return ax;
}

// ============================================================
//  图表绘制器
// ============================================================
class ChartRenderer {
    int W, H;
    Gdiplus::Bitmap* bmp;
    Gdiplus::Graphics* g;

    // 绘图区域 (像素)
    int ml = 80, mr = 30, mt = 50, mb = 55; // margins

    Gdiplus::RectF plotArea() const {
        return Gdiplus::RectF((float)ml, (float)mt,
                              (float)(W - ml - mr), (float)(H - mt - mb));
    }

    float mapX(double val, double xlo, double xhi) const {
        auto pa = plotArea();
        return pa.X + (float)((val - xlo) / (xhi - xlo)) * pa.Width;
    }
    float mapY(double val, double ylo, double yhi) const {
        auto pa = plotArea();
        return pa.Y + pa.Height - (float)((val - ylo) / (yhi - ylo)) * pa.Height;
    }

    void drawGrid(const AxisTicks& yax, double xlo, double xhi, int N) {
        auto pa = plotArea();
        Gdiplus::Pen gridPen(Gdiplus::Color(40, 0, 0, 0), 1.0f);
        gridPen.SetDashStyle(Gdiplus::DashStyleDot);

        Gdiplus::Font tickFont(L"Arial", 9);
        Gdiplus::SolidBrush textBr(Gdiplus::Color(60, 60, 60));
        Gdiplus::StringFormat sfRight;
        sfRight.SetAlignment(Gdiplus::StringAlignmentFar);
        sfRight.SetLineAlignment(Gdiplus::StringAlignmentCenter);
        Gdiplus::StringFormat sfCenter;
        sfCenter.SetAlignment(Gdiplus::StringAlignmentCenter);
        sfCenter.SetLineAlignment(Gdiplus::StringAlignmentNear);

        // Y grid + labels
        for (auto yv : yax.ticks) {
            float y = mapY(yv, yax.lo, yax.hi);
            g->DrawLine(&gridPen, pa.X, y, pa.X + pa.Width, y);
            wchar_t buf[32];
            // 自动选择格式
            if (std::abs(yv) < 1.0) swprintf(buf, 32, L"%.4f", yv);
            else if (std::abs(yv) < 100.0) swprintf(buf, 32, L"%.2f", yv);
            else swprintf(buf, 32, L"%.1f", yv);
            Gdiplus::RectF lr(0, y - 10, (float)(ml - 5), 20);
            g->DrawString(buf, -1, &tickFont, lr, &sfRight, &textBr);
        }

        // X labels (1..N)
        for (int i = 1; i <= N; i++) {
            float x = mapX(i, xlo, xhi);
            wchar_t buf[8];
            swprintf(buf, 8, L"%d", i);
            Gdiplus::RectF lr(x - 15, pa.Y + pa.Height + 3, 30, 20);
            g->DrawString(buf, -1, &tickFont, lr, &sfCenter, &textBr);
        }

        // 绘图区域边框
        Gdiplus::Pen borderPen(Gdiplus::Color(100, 100, 100), 1.0f);
        g->DrawRectangle(&borderPen, pa.X, pa.Y, pa.Width, pa.Height);
    }

public:
    ChartRenderer(int w = 700, int h = 400) : W(w), H(h) {
        bmp = new Gdiplus::Bitmap(W, H, PixelFormat32bppARGB);
        g = Gdiplus::Graphics::FromImage(bmp);
        g->SetSmoothingMode(Gdiplus::SmoothingModeAntiAlias);
        g->SetTextRenderingHint(Gdiplus::TextRenderingHintAntiAlias);
        g->Clear(Gdiplus::Color(255, 255, 255, 255));
    }

    ~ChartRenderer() {
        delete g;
        delete bmp;
    }

    // 柱状图
    void drawBarChart(const std::vector<double>& data,
                      const std::string& title,
                      const std::string& xlabel,
                      const std::string& ylabel,
                      Gdiplus::Color barColor,
                      const std::string& outPath)
    {
        int N = (int)data.size();
        double mn = vec_mean(data);

        // Y axis
        double margin = (vec_max(data) - vec_min(data)) * 0.3;
        if (margin < 0.001) margin = vec_max(data) * 0.15;
        double ymin_raw = std::max(0.0, vec_min(data) - margin);
        double ymax_raw = vec_max(data) + margin;
        AxisTicks yax = calc_ticks(ymin_raw, ymax_raw, 6);

        double xlo = 0.3, xhi = N + 0.7;

        // Title
        Gdiplus::Font titleFont(L"SimHei", 13, Gdiplus::FontStyleBold);
        Gdiplus::SolidBrush blackBr(Gdiplus::Color(0, 0, 0));
        Gdiplus::StringFormat sfCenter;
        sfCenter.SetAlignment(Gdiplus::StringAlignmentCenter);
        auto tw = to_wstr(title);
        Gdiplus::RectF titleRect(0, 5, (float)W, (float)mt);
        g->DrawString(tw.c_str(), -1, &titleFont, titleRect, &sfCenter, &blackBr);

        // Grid
        drawGrid(yax, xlo, xhi, N);
        auto pa = plotArea();

        // Bars
        Gdiplus::SolidBrush barBr(barColor);
        float barW = pa.Width / (float)(N + 1) * 0.55f;
        for (int i = 0; i < N; i++) {
            float cx = mapX(i + 1, xlo, xhi);
            float top = mapY(data[i], yax.lo, yax.hi);
            float bot = mapY(yax.lo, yax.lo, yax.hi);
            g->FillRectangle(&barBr, cx - barW / 2, top, barW, bot - top);
        }

        // Mean line
        float my = mapY(mn, yax.lo, yax.hi);
        Gdiplus::Pen meanPen(Gdiplus::Color(220, 30, 30), 1.5f);
        meanPen.SetDashStyle(Gdiplus::DashStyleDash);
        g->DrawLine(&meanPen, pa.X, my, pa.X + pa.Width, my);

        // Mean label
        char mbuf[64];
        sprintf(mbuf, "mean=%.4fmm", mn);
        auto mws = to_wstr(mbuf);
        Gdiplus::Font mf(L"Arial", 8);
        Gdiplus::SolidBrush redBr(Gdiplus::Color(220, 30, 30));
        g->DrawString(mws.c_str(), -1, &mf, Gdiplus::PointF(pa.X + pa.Width - 120, my - 16), &redBr);

        // Axis labels
        Gdiplus::Font labelFont(L"SimHei", 10);
        auto xlw = to_wstr(xlabel);
        Gdiplus::RectF xlRect(0, (float)(H - 22), (float)W, 20);
        g->DrawString(xlw.c_str(), -1, &labelFont, xlRect, &sfCenter, &blackBr);

        auto ylw = to_wstr(ylabel);
        // 竖向标签 - 旋转绘制
        Gdiplus::GraphicsState state = g->Save();
        g->TranslateTransform(14, (float)(mt + (H - mt - mb) / 2));
        g->RotateTransform(-90);
        Gdiplus::StringFormat sfCen2;
        sfCen2.SetAlignment(Gdiplus::StringAlignmentCenter);
        sfCen2.SetLineAlignment(Gdiplus::StringAlignmentCenter);
        Gdiplus::RectF ylRect(-100, -10, 200, 20);
        g->DrawString(ylw.c_str(), -1, &labelFont, ylRect, &sfCen2, &blackBr);
        g->Restore(state);

        // Save
        savePng(outPath);
        printf("  -> %s\n", outPath.c_str());
    }

    // 折线图 (带均值线和 ±σ 带)
    void drawLineChart(const std::vector<double>& data,
                       const std::string& title,
                       const std::string& xlabel,
                       const std::string& ylabel,
                       const std::string& outPath)
    {
        int N = (int)data.size();
        double mn = vec_mean(data);
        double sd = vec_std(data);

        double ymargin = 0.005;
        double ymin_raw = vec_min(data) - ymargin;
        double ymax_raw = vec_max(data) + ymargin;
        AxisTicks yax = calc_ticks(ymin_raw, ymax_raw, 6);

        double xlo = 0.3, xhi = N + 0.7;

        // Title
        Gdiplus::Font titleFont(L"SimHei", 13, Gdiplus::FontStyleBold);
        Gdiplus::SolidBrush blackBr(Gdiplus::Color(0, 0, 0));
        Gdiplus::StringFormat sfCenter;
        sfCenter.SetAlignment(Gdiplus::StringAlignmentCenter);
        auto tw = to_wstr(title);
        Gdiplus::RectF titleRect(0, 5, (float)W, (float)mt);
        g->DrawString(tw.c_str(), -1, &titleFont, titleRect, &sfCenter, &blackBr);

        drawGrid(yax, xlo, xhi, N);
        auto pa = plotArea();

        // ±σ 带
        float bandTop = mapY(mn + sd, yax.lo, yax.hi);
        float bandBot = mapY(mn - sd, yax.lo, yax.hi);
        // Clip to plot area
        bandTop = (std::max)(bandTop, pa.Y);
        bandBot = (std::min)(bandBot, pa.Y + pa.Height);
        Gdiplus::SolidBrush bandBr(Gdiplus::Color(30, 220, 30, 30));
        g->FillRectangle(&bandBr, pa.X, bandTop, pa.Width, bandBot - bandTop);

        // Mean line
        float my = mapY(mn, yax.lo, yax.hi);
        Gdiplus::Pen meanPen(Gdiplus::Color(220, 30, 30), 1.5f);
        meanPen.SetDashStyle(Gdiplus::DashStyleDash);
        g->DrawLine(&meanPen, pa.X, my, pa.X + pa.Width, my);

        // Data line
        Gdiplus::Pen linePen(Gdiplus::Color(51, 102, 204), 2.0f);
        for (int i = 0; i < N - 1; i++) {
            float x1 = mapX(i + 1, xlo, xhi), y1 = mapY(data[i], yax.lo, yax.hi);
            float x2 = mapX(i + 2, xlo, xhi), y2 = mapY(data[i + 1], yax.lo, yax.hi);
            g->DrawLine(&linePen, x1, y1, x2, y2);
        }

        // Data points
        Gdiplus::SolidBrush ptBr(Gdiplus::Color(51, 102, 204));
        for (int i = 0; i < N; i++) {
            float x = mapX(i + 1, xlo, xhi), y = mapY(data[i], yax.lo, yax.hi);
            g->FillEllipse(&ptBr, x - 4, y - 4, 8.0f, 8.0f);
        }

        // Legend
        float lx = pa.X + pa.Width - 175, ly = pa.Y + 8;
        Gdiplus::Pen lBorder(Gdiplus::Color(180, 180, 180));
        g->DrawRectangle(&lBorder, lx, ly, 170.0f, 50.0f);
        Gdiplus::SolidBrush lbg(Gdiplus::Color(240, 255, 255, 255));
        g->FillRectangle(&lbg, lx + 1, ly + 1, 168.0f, 48.0f);

        Gdiplus::Font lf(L"Arial", 8);
        Gdiplus::SolidBrush blueBr(Gdiplus::Color(51, 102, 204));
        Gdiplus::SolidBrush redBr(Gdiplus::Color(220, 30, 30));
        g->FillEllipse(&blueBr, lx + 8, ly + 8, 8.0f, 8.0f);
        g->DrawString(L"coplanarity", -1, &lf, Gdiplus::PointF(lx + 20, ly + 5), &blackBr);

        char buf1[64]; sprintf(buf1, "mean=%.4fmm", mn);
        g->DrawLine(&meanPen, lx + 5, ly + 26, lx + 18, ly + 26);
        g->DrawString(to_wstr(buf1).c_str(), -1, &lf, Gdiplus::PointF(lx + 20, ly + 20), &redBr);

        char buf2[64]; sprintf(buf2, "±σ (%.4fmm)", sd);
        Gdiplus::SolidBrush bandLBr(Gdiplus::Color(80, 220, 30, 30));
        g->FillRectangle(&bandLBr, lx + 5, ly + 37, 13.0f, 8.0f);
        g->DrawString(to_wstr(buf2).c_str(), -1, &lf, Gdiplus::PointF(lx + 20, ly + 34), &redBr);

        // Axis labels
        Gdiplus::Font labelFont(L"SimHei", 10);
        auto xlw = to_wstr(xlabel);
        Gdiplus::RectF xlRect(0, (float)(H - 22), (float)W, 20);
        g->DrawString(xlw.c_str(), -1, &labelFont, xlRect, &sfCenter, &blackBr);

        auto ylw = to_wstr(ylabel);
        Gdiplus::GraphicsState state2 = g->Save();
        g->TranslateTransform(14, (float)(mt + (H - mt - mb) / 2));
        g->RotateTransform(-90);
        Gdiplus::StringFormat sfCen2;
        sfCen2.SetAlignment(Gdiplus::StringAlignmentCenter);
        sfCen2.SetLineAlignment(Gdiplus::StringAlignmentCenter);
        Gdiplus::RectF ylRect(-100, -10, 200, 20);
        g->DrawString(ylw.c_str(), -1, &labelFont, ylRect, &sfCen2, &blackBr);
        g->Restore(state2);

        savePng(outPath);
        printf("  -> %s\n", outPath.c_str());
    }

    // 重置画布
    void reset() {
        delete g;
        delete bmp;
        bmp = new Gdiplus::Bitmap(W, H, PixelFormat32bppARGB);
        g = Gdiplus::Graphics::FromImage(bmp);
        g->SetSmoothingMode(Gdiplus::SmoothingModeAntiAlias);
        g->SetTextRenderingHint(Gdiplus::TextRenderingHintAntiAlias);
        g->Clear(Gdiplus::Color(255, 255, 255, 255));
    }

    // 双色阈值柱状图 (BGA焊球偏差分布)
    // 绝对值 < threshold 为 normalColor, 否则为 alertColor
    void drawThresholdBarChart(const std::vector<double>& data,
                               const std::string& title,
                               const std::string& xlabel,
                               const std::string& ylabel,
                               double threshold,
                               Gdiplus::Color normalColor,
                               Gdiplus::Color alertColor,
                               const std::string& outPath)
    {
        int N = (int)data.size();

        // 取绝对值用于绘图
        std::vector<double> absData(N);
        for (int i = 0; i < N; i++) absData[i] = std::abs(data[i]);
        double mn = vec_mean(absData);

        // Y axis
        double margin = (vec_max(absData) - vec_min(absData)) * 0.3;
        if (margin < 0.001) margin = vec_max(absData) * 0.15;
        double ymin_raw = 0.0;
        double ymax_raw = vec_max(absData) + margin;
        AxisTicks yax = calc_ticks(ymin_raw, ymax_raw, 6);

        double xlo = 0.0, xhi = N + 1.0;

        // Title
        Gdiplus::Font titleFont(L"SimHei", 13, Gdiplus::FontStyleBold);
        Gdiplus::SolidBrush blackBr(Gdiplus::Color(0, 0, 0));
        Gdiplus::StringFormat sfCenter;
        sfCenter.SetAlignment(Gdiplus::StringAlignmentCenter);
        auto tw = to_wstr(title);
        Gdiplus::RectF titleRect(0, 5, (float)W, (float)mt);
        g->DrawString(tw.c_str(), -1, &titleFont, titleRect, &sfCenter, &blackBr);

        // Grid
        drawGrid(yax, xlo, xhi, N);
        auto pa = plotArea();

        // Bars with threshold coloring
        Gdiplus::SolidBrush normBr(normalColor);
        Gdiplus::SolidBrush alertBr(alertColor);
        float barW = pa.Width / (float)(N + 1) * 0.7f;
        if (barW > 12.0f) barW = 12.0f;  // 48球时柱子不要太宽
        for (int i = 0; i < N; i++) {
            float cx = mapX(i + 1, xlo, xhi);
            float top = mapY(absData[i], yax.lo, yax.hi);
            float bot = mapY(yax.lo, yax.lo, yax.hi);
            if (absData[i] < threshold)
                g->FillRectangle(&normBr, cx - barW / 2, top, barW, bot - top);
            else
                g->FillRectangle(&alertBr, cx - barW / 2, top, barW, bot - top);
        }

        // Mean line
        float my = mapY(mn, yax.lo, yax.hi);
        Gdiplus::Pen meanPen(Gdiplus::Color(220, 30, 30), 1.5f);
        meanPen.SetDashStyle(Gdiplus::DashStyleDash);
        g->DrawLine(&meanPen, pa.X, my, pa.X + pa.Width, my);

        // Mean label
        char mbuf[64];
        sprintf(mbuf, "mean=%.4fmm", mn);
        auto mws = to_wstr(mbuf);
        Gdiplus::Font mf(L"Arial", 8);
        Gdiplus::SolidBrush redBr(Gdiplus::Color(220, 30, 30));
        g->DrawString(mws.c_str(), -1, &mf, Gdiplus::PointF(pa.X + pa.Width - 120, my - 16), &redBr);

        // Axis labels
        Gdiplus::Font labelFont(L"SimHei", 10);
        auto xlw = to_wstr(xlabel);
        Gdiplus::RectF xlRect(0, (float)(H - 22), (float)W, 20);
        g->DrawString(xlw.c_str(), -1, &labelFont, xlRect, &sfCenter, &blackBr);

        auto ylw = to_wstr(ylabel);
        Gdiplus::GraphicsState state3 = g->Save();
        g->TranslateTransform(14, (float)(mt + (H - mt - mb) / 2));
        g->RotateTransform(-90);
        Gdiplus::StringFormat sfCen3;
        sfCen3.SetAlignment(Gdiplus::StringAlignmentCenter);
        sfCen3.SetLineAlignment(Gdiplus::StringAlignmentCenter);
        Gdiplus::RectF ylRect3(-100, -10, 200, 20);
        g->DrawString(ylw.c_str(), -1, &labelFont, ylRect3, &sfCen3, &blackBr);
        g->Restore(state3);

        savePng(outPath);
        printf("  -> %s\n", outPath.c_str());
    }

private:
    void savePng(const std::string& path) {
        CLSID clsid;
        GetEncoderClsid(L"image/png", &clsid);
        auto wp = to_wstr(path);
        bmp->Save(wp.c_str(), &clsid, nullptr);
    }
};

#endif // CHART_BINDGEN_HPP
