% qfp_coplanarity_main.m - QFP芯片引脚共面度测量主程序
% 基于论文第5.3.2节理论实现
% 包含4个层次化测量步骤：
% (1) 封装本体顶面曲面拟合与法向量提取
% (2) 基于高度分层的引脚底部接触区提取
% (3) 基于法向量约束与空间连通性的点云精化
% (4) 引脚底部接触面平面拟合与共面度计算

clear; clc; close all;

%% 添加路径
addpath('f:\cs_project\3D\光线回溯法-远心标定\远心单目双光栅\交比不变性拟合\交比不变性重建\Chip\four\pointCal');
addpath('f:\cs_project\3D\光线回溯法-远心标定\远心单目双光栅\交比不变性拟合\交比不变性重建\Chip\four\segment');

fprintf('======================================\n');
fprintf('QFP芯片引脚共面度测量算法\n');
fprintf('======================================\n\n');

%% 读取必要的输入数据
% 需要：
% 1. X.mat, Y.mat, Z.mat - 融合点云数据（图像尺寸的矩阵）
% 2. mask_chip - 封装本体掩膜（来自引脚定位步骤）
% 3. pin_regions - 引脚区域信息（来自引脚定位步骤）

fprintf('加载输入数据...\n');

%% 1. 加载点云数据
X_path = 'X.mat';
Y_path = 'Y.mat';
Z_path = 'Z.mat';

if exist(X_path, 'file') && exist(Y_path, 'file') && exist(Z_path, 'file')
    fprintf('加载点云数据:\n');
    fprintf('  %s\n', X_path);
    fprintf('  %s\n', Y_path);
    fprintf('  %s\n', Z_path);
    load(X_path, 'X');
    load(Y_path, 'Y');
    load(Z_path, 'Z');
    [H, W] = size(Z);
    fprintf('  点云尺寸: %d × %d\n', H, W);
else
    error(['未找到点云文件！请确保以下文件存在：\n' ...
           '  %s\n  %s\n  %s'], X_path, Y_path, Z_path);
end

%% 2. 加载引脚定位结果
segment_result_path = 'f:\cs_project\3D\光线回溯法-远心标定\远心单目双光栅\交比不变性拟合\交比不变性重建\Chip\four\segment\results\qfp_pin_localization_results.mat';

if exist(segment_result_path, 'file')
    fprintf('\n加载引脚定位结果: %s\n', segment_result_path);
    load(segment_result_path, 'mask_chip', 'pin_regions');
    mask_body = mask_chip;  % 芯片掩膜作为本体掩膜
    
    % 验证掩膜尺寸与点云一致
    if ~isequal(size(mask_body), [H, W])
        error('掩膜尺寸 (%d×%d) 与点云尺寸 (%d×%d) 不一致！', ...
              size(mask_body,1), size(mask_body,2), H, W);
    end
else
    error(['未找到引脚定位结果！请先运行test_qfp_all.m生成：\n' ...
           '  %s'], segment_result_path);
end

%% 3. 验证数据完整性
fprintf('\n数据加载完成\n');
fprintf('  封装本体像素数: %d\n', sum(mask_body(:)));

% 统计引脚数量
sides = {'top', 'bottom', 'left', 'right'};
n_pins_total = 0;
for s = 1:length(sides)
    side_name = sides{s};
    if isfield(pin_regions, side_name)
        if isfield(pin_regions.(side_name), 'num_pins')
            n_pins_side = pin_regions.(side_name).num_pins;
        elseif isfield(pin_regions.(side_name), 'pins')
            n_pins_side = length(pin_regions.(side_name).pins);
        else
            n_pins_side = 0;
        end
        fprintf('  %s侧引脚数: %d\n', side_name, n_pins_side);
        n_pins_total = n_pins_total + n_pins_side;
    end
end
fprintf('  引脚总数: %d\n', n_pins_total);

% 检查有效点云
valid_points = (Z ~= 0) & ~isnan(Z);
fprintf('  有效点云数: %d (%.1f%%)\n', sum(valid_points(:)), ...
        100*sum(valid_points(:))/numel(Z));
fprintf('  Z值范围: [%.4f, %.4f] mm\n\n', ...
        min(Z(valid_points)), max(Z(valid_points)));

%% 步骤1: 封装本体顶面曲面拟合与法向量提取
fprintf('====== 步骤1/4: 封装本体曲面拟合 ======\n');
body_surface = QFP_Cal_Step1_BodySurfaceFit(X, Y, Z, mask_body);
fprintf('本体曲面拟合完成\n');
fprintf('  法向量: [%.4f, %.4f, %.4f]\n', body_surface.normal_vector);
fprintf('  RMSE: %.4f mm\n\n', body_surface.rmse);
pause(0.5);

%% 步骤2: 基于高度分层的引脚底部接触区提取
fprintf('====== 步骤2/4: 高度分层提取底部点云 ======\n');
params_height = struct('k_z', 2.0, 'n_bins', 50);
pins_bottom_candidates = QFP_Cal_Step2_PinTipExtraction(X, Y, Z, pin_regions, params_height);
fprintf('高度分层提取完成\n');
fprintf('  有效引脚数: %d\n\n', length(pins_bottom_candidates));
pause(0.5);

%% 步骤3: 基于法向量约束与空间连通性的点云精化
fprintf('====== 步骤3/4: 法向量约束精化 ======\n');
params_normal = struct('r_local', 0.2, 'theta_thr', 30, 'cluster_dist', 0.3);
pins_bottom_refined = QFP_Cal_Step3_PointCloudRefine(pins_bottom_candidates, body_surface, params_normal);
fprintf('法向量约束精化完成\n');
fprintf('  精化引脚数: %d\n\n', length(pins_bottom_refined));
pause(0.5);

%% 步骤4: 引脚底部接触面平面拟合与共面度计算
fprintf('====== 步骤4/4: 平面拟合与共面度计算 ======\n');
params_fitting = struct('max_iter', 5, 'kappa', 1.5);
coplanarity_result = QFP_Cal_Step4_CoplanarityCalc(pins_bottom_refined, params_fitting);
fprintf('平面拟合与共面度计算完成\n\n');
pause(0.5);

%% 保存结果
output_dir = 'f:\cs_project\3D\光线回溯法-远心标定\远心单目双光栅\交比不变性拟合\交比不变性重建\Chip\four\pointCal\results';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fprintf('保存结果...\n');
save(fullfile(output_dir, 'qfp_coplanarity_results.mat'), ...
    'X', 'Y', 'Z', 'mask_body', 'pin_regions', ...
    'body_surface', 'pins_bottom_candidates', 'pins_bottom_refined', ...
    'coplanarity_result');
fprintf('  MAT文件已保存\n');

% 保存共面度测量报告
report_file = fullfile(output_dir, 'coplanarity_report.txt');
fid = fopen(report_file, 'w');
fprintf(fid, '======================================\n');
fprintf(fid, 'QFP芯片引脚共面度测量报告\n');
fprintf(fid, '======================================\n\n');
fprintf(fid, '测量日期: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

fprintf(fid, '1. 封装本体信息:\n');
fprintf(fid, '   法向量: [%.6f, %.6f, %.6f]\n', body_surface.normal_vector);
fprintf(fid, '   曲面拟合RMSE: %.4f mm\n\n', body_surface.rmse);

fprintf(fid, '2. 引脚测量统计:\n');
fprintf(fid, '   检测引脚总数: %d\n', length(coplanarity_result.pins));
for s = 1:length(sides)
    side_name = sides{s};
    if ~isempty(coplanarity_result.statistics_by_side.(side_name))
        st = coplanarity_result.statistics_by_side.(side_name);
        fprintf(fid, '   %s侧: %d个引脚\n', side_name, st.count);
    end
end
fprintf(fid, '\n');

fprintf(fid, '3. 共面度测量结果:\n');
fprintf(fid, '   共面度特征值: %.4f mm\n', coplanarity_result.coplanarity_value);
fprintf(fid, '   最大正偏差: %.4f mm\n', coplanarity_result.max_deviation);
fprintf(fid, '   最大负偏差: %.4f mm\n', coplanarity_result.min_deviation);
fprintf(fid, '   偏差标准差: %.4f mm\n\n', coplanarity_result.deviation_std);

fprintf(fid, '4. 各侧偏差统计:\n');
for s = 1:length(sides)
    side_name = sides{s};
    if ~isempty(coplanarity_result.statistics_by_side.(side_name))
        st = coplanarity_result.statistics_by_side.(side_name);
        fprintf(fid, '   %s侧:\n', side_name);
        fprintf(fid, '     引脚数: %d\n', st.count);
        fprintf(fid, '     平均偏差: %.4f mm\n', st.mean_dev);
        fprintf(fid, '     标准差: %.4f mm\n', st.std_dev);
        fprintf(fid, '     偏差范围: [%.4f, %.4f] mm\n\n', st.min_dev, st.max_dev);
    end
end

fprintf(fid, '5. 各引脚详细信息:\n');
fprintf(fid, 'ID\t侧\t顶点X(mm)\t顶点Y(mm)\t顶点Z(mm)\t偏差(mm)\tRMSE(mm)\t点数\n');
for i = 1:length(coplanarity_result.pins)
    pin = coplanarity_result.pins(i);
    fprintf(fid, '%d\t%s\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%d\n', ...
            pin.pin_id, pin.side, pin.vertex(1), pin.vertex(2), pin.vertex(3), ...
            pin.deviation, pin.rmse, pin.n_points);
end

fclose(fid);
fprintf('  测量报告已保存: coplanarity_report.txt\n');

%% 生成综合结果图
fprintf('\n生成综合结果可视化...\n');
figure('Position', [50, 50, 1600, 900], 'Color', 'w');

% 提取所有顶点
vertices = zeros(length(coplanarity_result.pins), 3);
deviations = zeros(length(coplanarity_result.pins), 1);
for i = 1:length(coplanarity_result.pins)
    vertices(i, :) = coplanarity_result.pins(i).vertex;
    deviations(i) = coplanarity_result.pins(i).deviation;
end

% 子图1: 封装本体曲面
subplot(2, 3, 1);
pc_body = body_surface.point_cloud;
scatter3(pc_body(:,1), pc_body(:,2), pc_body(:,3), 10, pc_body(:,3), 'filled');
colormap(gca, jet);
title('步骤1: 本体曲面拟合', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)');
axis equal; grid on; view(45, 30);

% 子图2: 高度分层（第一个引脚示例）
subplot(2, 3, 2);
if ~isempty(pins_bottom_candidates)
    pin_ex = pins_bottom_candidates(1);
    scatter3(pin_ex.point_cloud_full(:,1), pin_ex.point_cloud_full(:,2), ...
             pin_ex.point_cloud_full(:,3), 15, [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.3);
    hold on;
    scatter3(pin_ex.point_cloud_bottom(:,1), pin_ex.point_cloud_bottom(:,2), ...
             pin_ex.point_cloud_bottom(:,3), 30, 'b', 'filled');
    title('步骤2: 高度分层提取', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
    xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)');
    legend({'完整点云', '底部候选'}, 'FontSize', 9, 'FontName', 'SimHei');
    axis equal; grid on; view(45, 30);
end

% 子图3: 法向量约束（第一个引脚示例）
subplot(2, 3, 3);
if ~isempty(pins_bottom_refined)
    pin_ex = pins_bottom_refined(1);
    scatter3(pin_ex.point_cloud_bottom_candidate(:,1), ...
             pin_ex.point_cloud_bottom_candidate(:,2), ...
             pin_ex.point_cloud_bottom_candidate(:,3), ...
             15, [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.3);
    hold on;
    scatter3(pin_ex.point_cloud_bottom_refined(:,1), ...
             pin_ex.point_cloud_bottom_refined(:,2), ...
             pin_ex.point_cloud_bottom_refined(:,3), 30, 'g', 'filled');
    title('步骤3: 法向量精化', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
    xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)');
    legend({'候选点', '精化点'}, 'FontSize', 9, 'FontName', 'SimHei');
    axis equal; grid on; view(45, 30);
end

% 子图4: 平面拟合（第一个引脚示例）
subplot(2, 3, 4);
if ~isempty(coplanarity_result.pins)
    pin_ex = coplanarity_result.pins(1);
    scatter3(pin_ex.point_cloud(:,1), pin_ex.point_cloud(:,2), ...
             pin_ex.point_cloud(:,3), 20, 'b', 'filled');
    hold on;
    plot3(pin_ex.vertex(1), pin_ex.vertex(2), pin_ex.vertex(3), ...
          'r*', 'MarkerSize', 20, 'LineWidth', 3);
    title('步骤4: 平面拟合', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
    xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)');
    legend({'点云', '顶点'}, 'FontSize', 9, 'FontName', 'SimHei');
    axis equal; grid on; view(45, 30);
end

% 子图5: 顶点分布与基准平面
subplot(2, 3, 5);
ref_plane = coplanarity_result.reference_plane;
x_range = [min(vertices(:,1)), max(vertices(:,1))];
y_range = [min(vertices(:,2)), max(vertices(:,2))];
[X_grid, Y_grid] = meshgrid(linspace(x_range(1), x_range(2), 20), ...
                             linspace(y_range(1), y_range(2), 20));
Z_grid = ref_plane(1)*X_grid + ref_plane(2)*Y_grid + ref_plane(3);
surf(X_grid, Y_grid, Z_grid, 'FaceAlpha', 0.3, 'EdgeColor', [0.7 0.7 0.7], 'FaceColor', [0.9 0.9 1]);
hold on;
scatter3(vertices(:,1), vertices(:,2), vertices(:,3), 80, deviations, 'filled', 'MarkerEdgeColor', 'k');
colormap(jet);
title('顶点与基准平面', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)');
colorbar; axis equal; grid on; view(45, 30);

% 子图6: 偏差分布
subplot(2, 3, 6);
histogram(deviations, 15, 'FaceColor', [0.3 0.7 0.9], 'EdgeColor', 'k');
xlabel('偏差(mm)', 'FontSize', 11, 'FontName', 'SimHei');
ylabel('引脚数', 'FontSize', 11, 'FontName', 'SimHei');
title(sprintf('共面度分布\nC=%.4fmm', coplanarity_result.coplanarity_value), ...
      'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;

sgtitle('QFP芯片引脚共面度测量完整流程', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 保存综合结果图
saveas(gcf, fullfile(output_dir, 'coplanarity_measurement_overview.png'));
fprintf('  综合结果图已保存\n');

%% 显示最终统计信息
fprintf('\n======================================\n');
fprintf('共面度测量完成\n');
fprintf('======================================\n');
fprintf('测量引脚总数: %d\n', length(coplanarity_result.pins));
fprintf('共面度特征值: %.4f mm\n', coplanarity_result.coplanarity_value);
fprintf('最大正偏差: %.4f mm\n', coplanarity_result.max_deviation);
fprintf('最大负偏差: %.4f mm\n', coplanarity_result.min_deviation);
fprintf('偏差标准差: %.4f mm\n', coplanarity_result.deviation_std);

fprintf('\n各侧统计:\n');
for s = 1:length(sides)
    side_name = sides{s};
    if isfield(coplanarity_result.statistics_by_side, side_name) && ...
       ~isempty(coplanarity_result.statistics_by_side.(side_name))
        st = coplanarity_result.statistics_by_side.(side_name);
        fprintf('  %s侧: %d个, 均值=%.4f±%.4fmm\n', ...
                side_name, st.count, st.mean_dev, st.std_dev);
    end
end

fprintf('\n所有结果已保存至:\n%s\n', output_dir);
fprintf('======================================\n');
