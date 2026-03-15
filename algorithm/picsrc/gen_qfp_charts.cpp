/*
 * gen_qfp_charts.cpp - 生成QFP共面度/翘曲度相关图表
 *
 * 从 CSV 文件读取10次测量数据，生成:
 *   output/qfp_coplanarity.png      - QFP共面度柱状图
 *   output/qfp_max_warp_angle.png   - 最大翘曲角柱状图
 *   output/qfp_avg_warp_angle.png   - 平均翘曲角柱状图
 *   output/qfp_max_warp_height.png  - 最大翘曲高度柱状图
 *
 * 编译: cl /EHsc /utf-8 /std:c++17 gen_qfp_charts.cpp /link gdiplus.lib
 */

#include "chart_bindgen.hpp"
#include <cstdio>

int main(int argc, char* argv[]) {
    std::string data_dir   = "..\\data";
    std::string output_dir = "output";
    if (argc >= 3) {
        data_dir   = argv[1];
        output_dir = argv[2];
    }

    std::string csv_path = data_dir + "\\qfp_measurements.csv";

    printf("[gen_qfp_charts] Reading %s ...\n", csv_path.c_str());

    CsvTable tbl = read_csv(csv_path);
    if (tbl.rows() == 0 || tbl.cols() < 5) {
        fprintf(stderr, "ERROR: Invalid QFP data (need 5 columns).\n");
        return 1;
    }

    std::vector<double> cop    = tbl.col(1); // 共面度
    std::vector<double> mxAng  = tbl.col(2); // 最大翘曲角
    std::vector<double> mnAng  = tbl.col(3); // 平均翘曲角
    std::vector<double> mxH    = tbl.col(4); // 最大翘曲高度

    printf("  Loaded %d QFP measurements\n", (int)cop.size());

    GdiplusSession gdi;

    // 图1: QFP共面度
    {
        ChartRenderer cr(650, 450);
        cr.drawBarChart(cop,
                        u8"QFP共面度10次重复测量",
                        u8"测量次数",
                        u8"共面度/mm",
                        Gdiplus::Color(33, 150, 243),   // #2196F3 蓝
                        output_dir + "\\qfp_coplanarity.png");
    }

    // 图2: 最大翘曲角
    {
        ChartRenderer cr(650, 450);
        cr.drawBarChart(mxAng,
                        u8"最大翘曲角分布",
                        u8"测量次数",
                        u8"最大翘曲角/\u00b0",
                        Gdiplus::Color(156, 39, 176),   // #9C27B0 紫
                        output_dir + "\\qfp_max_warp_angle.png");
    }

    // 图3: 平均翘曲角
    {
        ChartRenderer cr(650, 450);
        cr.drawBarChart(mnAng,
                        u8"平均翘曲角分布",
                        u8"测量次数",
                        u8"平均翘曲角/\u00b0",
                        Gdiplus::Color(0, 186, 55),     // 绿
                        output_dir + "\\qfp_avg_warp_angle.png");
    }

    // 图4: 最大翘曲高度
    {
        ChartRenderer cr(650, 450);
        cr.drawBarChart(mxH,
                        u8"最大翘曲高度分布",
                        u8"测量次数",
                        u8"最大翘曲高度/mm",
                        Gdiplus::Color(233, 30, 99),    // #E91E63 粉红
                        output_dir + "\\qfp_max_warp_height.png");
    }

    printf("[gen_qfp_charts] Done.\n");
    return 0;
}
