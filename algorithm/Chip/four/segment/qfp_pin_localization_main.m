% qfp_pin_localization_main.m - QFP芯片引脚定位主程序
% 基于论文第5.2.2节理论实现
% 包含5个层次化定位步骤：
% (1) 粗略定位
% (2) 本体边界精细提取
% (3) 搜索区域划分
% (4) 多约束几何过滤
% (5) 基于投影的单个引脚分割

clear; clc; close all;

%% 添加路径
addpath('f:\cs_project\3D\光线回溯法-远心标定\远心单目双光栅\交比不变性拟合\交比不变性重建\Chip\four\segment');

%% 读取图像
img_path = 'fourchip.png';
fprintf('======================================\n');
fprintf('QFP芯片引脚定位算法\n');
fprintf('======================================\n\n');
fprintf('读取图像: %s\n', img_path);

if ~exist(img_path, 'file')
    error('图像文件不存在！');
end

I = imread(img_path);
if size(I, 3) == 3
    I_gray = rgb2gray(I);
else
    I_gray = I;
end
[H, W] = size(I_gray);
fprintf('图像尺寸: %d × %d\n\n', H, W);

%% 步骤1: 粗略定位
fprintf('====== 步骤1/5: 粗略定位 ======\n');
[I_chip, mask_chip] = QFP_Seg_Step1_ChipLocalization(I_gray);
fprintf('粗略定位完成\n\n');
pause(0.5);

%% 步骤2: 本体边界精细提取
fprintf('====== 步骤2/5: 本体边界精细提取 ======\n');
[body_boundary, body_mask] = QFP_Seg_Step2_BodyBoundary(I_gray, mask_chip);
fprintf('边界提取完成\n\n');
pause(0.5);

%% 步骤3: 搜索区域划分
fprintf('====== 步骤3/5: 搜索区域划分 ======\n');
search_regions = QFP_Seg_Step3_SearchRegion(I_gray, mask_chip, body_boundary);
fprintf('搜索区域划分完成\n\n');
pause(0.5);

%% 步骤4: 多约束几何过滤
fprintf('====== 步骤4/5: 多约束几何过滤 ======\n');
pins_regions = QFP_Seg_Step4_GeometricFilter(I_gray, search_regions, body_boundary);

% 统计过滤后的引脚数量
total_pins = 0;
sides = {'top', 'bottom', 'left', 'right'};
for s = 1:length(sides)
    side_name = sides{s};
    if isfield(pins_regions, side_name) && ~isempty(pins_regions.(side_name))
        n = length(pins_regions.(side_name));
        fprintf('  %s侧: %d个候选引脚\n', side_name, n);
        total_pins = total_pins + n;
    end
end
fprintf('几何过滤完成，共 %d 个候选引脚\n\n', total_pins);
pause(0.5);

%% 步骤5: 基于投影的单个引脚分割与编号
fprintf('====== 步骤5/5: 单个引脚分割与编号 ======\n');
[pins_individual, pins_masks] = QFP_Seg_Step5_PinSegmentation(I_gray, pins_regions);
fprintf('引脚分割与编号完成\n\n');
pause(0.5);

%% 保存结果
output_dir = 'f:\cs_project\3D\光线回溯法-远心标定\远心单目双光栅\交比不变性拟合\交比不变性重建\Chip\four\segment\results';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% 保存数据
save(fullfile(output_dir, 'qfp_pin_localization_results.mat'), ...
    'I_gray', 'mask_chip', 'body_boundary', 'body_mask', ...
    'search_regions', 'pins_regions', 'pins_individual', 'pins_masks');
fprintf('结果已保存至: %s\n', output_dir);

% 保存引脚信息表格
if isfield(pins_individual, 'all_pins') && ~isempty(pins_individual.all_pins)
    n_pins = length(pins_individual.all_pins);
    pin_table = table();
    pin_table.ID = (1:n_pins)';
    pin_table.Side = cell(n_pins, 1);
    pin_table.Centroid_X = zeros(n_pins, 1);
    pin_table.Centroid_Y = zeros(n_pins, 1);
    
    for i = 1:n_pins
        pin_table.Side{i} = pins_individual.all_pins(i).side;
        pin_table.Centroid_X(i) = pins_individual.all_pins(i).centroid(1);
        pin_table.Centroid_Y(i) = pins_individual.all_pins(i).centroid(2);
    end
    
    writetable(pin_table, fullfile(output_dir, 'pin_info.csv'));
    fprintf('引脚信息表已保存\n');
end

%% 生成综合结果图
figure('Position', [50, 50, 1600, 900], 'Color', 'w');

% 子图1: 原始图像
subplot(2, 3, 1);
imshow(I_gray);
title('原始图像', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 子图2: 粗略定位结果
subplot(2, 3, 2);
imshow(I_chip);
title('步骤1: 粗略定位', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 子图3: 本体边界
subplot(2, 3, 3);
imshow(I_gray); hold on;
if ~isempty(body_boundary)
    % 绘制四条边界线
    plot([1, W], [body_boundary.top.b, body_boundary.top.k*W+body_boundary.top.b], 'r-', 'LineWidth', 2);
    plot([1, W], [body_boundary.bottom.b, body_boundary.bottom.k*W+body_boundary.bottom.b], 'r-', 'LineWidth', 2);
    plot([body_boundary.left.b, body_boundary.left.k*H+body_boundary.left.b], [1, H], 'r-', 'LineWidth', 2);
    plot([body_boundary.right.b, body_boundary.right.k*H+body_boundary.right.b], [1, H], 'r-', 'LineWidth', 2);
end
title('步骤2: 本体边界', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 子图4: 搜索区域
subplot(2, 3, 4);
imshow(I_gray); hold on;
colors = [1 0 0; 0 1 0; 0 0 1; 1 1 0];
for s = 1:length(sides)
    side_name = sides{s};
    if isfield(search_regions, side_name) && ~isempty(search_regions.(side_name))
        mask_s = search_regions.(side_name).mask;
        h = imshow(repmat(mask_s, [1 1 3]) .* reshape(colors(s,:), [1 1 3]));
        set(h, 'AlphaData', mask_s * 0.3);
    end
end
title('步骤3: 搜索区域', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 子图5: 几何过滤后引脚
subplot(2, 3, 5);
imshow(I_gray); hold on;
for s = 1:length(sides)
    side_name = sides{s};
    if isfield(pins_regions, side_name) && ~isempty(pins_regions.(side_name))
        for i = 1:length(pins_regions.(side_name))
            boundary = pins_regions.(side_name)(i).BoundingBox;
            rectangle('Position', boundary, 'EdgeColor', 'g', 'LineWidth', 2);
        end
    end
end
title('步骤4: 几何过滤', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 子图6: 最终引脚编号
subplot(2, 3, 6);
imshow(I_gray); hold on;
if isfield(pins_individual, 'all_pins') && ~isempty(pins_individual.all_pins)
    for i = 1:length(pins_individual.all_pins)
        centroid = pins_individual.all_pins(i).centroid;
        text(centroid(1), centroid(2), sprintf('%d', pins_individual.all_pins(i).id), ...
            'Color', 'y', 'FontSize', 9, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'BackgroundColor', 'k');
        
        % 绘制引脚轮廓
        boundary = bwboundaries(pins_individual.all_pins(i).mask);
        if ~isempty(boundary)
            plot(boundary{1}(:,2), boundary{1}(:,1), 'g-', 'LineWidth', 1.5);
        end
    end
end
title(sprintf('步骤5: 引脚编号 (共%d个)', length(pins_individual.all_pins)), ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

sgtitle('QFP芯片引脚定位完整流程', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 保存综合结果图
saveas(gcf, fullfile(output_dir, 'qfp_pin_localization_overview.png'));
fprintf('综合结果图已保存\n\n');

%% 显示最终统计信息
fprintf('======================================\n');
fprintf('定位算法执行完成\n');
fprintf('======================================\n');
fprintf('检测到的引脚总数: %d\n', length(pins_individual.all_pins));

% 各侧引脚统计
for s = 1:length(sides)
    side_name = sides{s};
    count = sum(strcmp({pins_individual.all_pins.side}, side_name));
    fprintf('  %s侧: %d个\n', side_name, count);
end

fprintf('\n所有结果已保存至:\n%s\n', output_dir);
fprintf('======================================\n');
