/*
 * gen_bga_charts.cpp - 生成BGA共面度相关图表
 *
 * 从 CSV 文件读取数据，生成:
 *   output/bga_bar_chart.png       - BGA共面度柱状图 (10次)
 *   output/bga_line_chart.png      - BGA共面度10次重复测量折线图
 *   output/bga_ball_deviation.png  - BGA焊球共面度偏差分布 (48球)
 *
 * 编译: cl /EHsc /utf-8 /std:c++17 gen_bga_charts.cpp /link gdiplus.lib
 */

#include "chart_bindgen.hpp"
#include <cstdio>

int main(int argc, char* argv[]) {
    // 路径配置 (默认读取共享数据目录 code/data/)
    std::string data_dir   = "..\\data";
    std::string output_dir = "output";
    if (argc >= 3) {
        data_dir   = argv[1];
        output_dir = argv[2];
    }

    // 初始化 GDI+
    GdiplusSession gdi;

    // ── BGA 10次共面度 ──
    {
        std::string csv_path = data_dir + "\\bga_coplanarity.csv";
        printf("[gen_bga_charts] Reading %s ...\n", csv_path.c_str());

        CsvTable tbl = read_csv(csv_path);
        if (tbl.rows() == 0) {
            fprintf(stderr, "ERROR: No data loaded from %s\n", csv_path.c_str());
            return 1;
        }

        std::vector<double> cop = tbl.col(1);
        printf("  Loaded %d measurements, mean=%.4f, std=%.4f\n",
               (int)cop.size(), vec_mean(cop), vec_std(cop));

        // 图1: BGA共面度柱状图
        {
            ChartRenderer cr(700, 400);
            cr.drawBarChart(cop,
                            u8"BGA共面度测量结果",
                            u8"测量次数",
                            u8"共面度/mm",
                            Gdiplus::Color(76, 175, 80),
                            output_dir + "\\bga_bar_chart.png");
        }

        // 图2: BGA共面度10次重复测量折线图
        {
            ChartRenderer cr(700, 400);
            cr.drawLineChart(cop,
                             u8"BGA共面度10次重复测量",
                             u8"测量次数",
                             u8"共面度/mm",
                             output_dir + "\\bga_line_chart.png");
        }
    }

    // ── BGA 48球逐球偏差分布 ──
    {
        std::string csv_path = data_dir + "\\bga_ball_deviations.csv";
        printf("[gen_bga_charts] Reading %s ...\n", csv_path.c_str());

        CsvTable tbl = read_csv(csv_path);
        if (tbl.rows() == 0) {
            fprintf(stderr, "WARNING: No ball deviation data from %s, skipping.\n",
                    csv_path.c_str());
        } else {
            std::vector<double> dev = tbl.col(1);  // 有符号偏差
            printf("  Loaded %d balls\n", (int)dev.size());

            // 阈值: mean(|dev|) + 2*std(|dev|)
            std::vector<double> absdev(dev.size());
            for (size_t i = 0; i < dev.size(); i++) absdev[i] = std::abs(dev[i]);
            double mn = vec_mean(absdev);
            double sd = vec_std(absdev);
            double threshold = mn + 2.0 * sd;
            printf("  |dev| mean=%.4f, std=%.4f, threshold=%.4f\n", mn, sd, threshold);

            // 图3: BGA焊球共面度偏差分布
            ChartRenderer cr(700, 400);
            cr.drawThresholdBarChart(dev,
                u8"BGA焊球共面度偏差分布",
                u8"焊球编号",
                u8"共面度偏差/mm",
                threshold,
                Gdiplus::Color(76, 175, 80),   // 绿: 正常
                Gdiplus::Color(255, 87, 34),   // 橙红: 超阈值
                output_dir + "\\bga_ball_deviation.png");
        }
    }

    printf("[gen_bga_charts] Done.\n");
    return 0;
}
