% pic5_05.m - 图5-5: 针对过曝边缘的迭代剔除圆拟合策略
% (a) Canny提取的原始边缘点
% (b) 第1轮迭代剔除虚假点
% (c) 第3轮收敛结果
% (d) 亚像素级圆心定位精度验证
% 输入: bga.bmp

clear; clc; close all;

%% 读取原始图像
I_original = imread('bga.bmp');
if size(I_original, 3) == 3
    I_original = rgb2gray(I_original);
end
% load("matlab.mat");
% I_original = bga_image;
%% 芯片区域粗定位前置步骤
% 1. Otsu全局二值分割
level_chip = graythresh(I_original);
% level_chip = 0.2;
BW_chip = imbinarize(I_original, level_chip);

% 2. 形态学操作：先腐蚀后膨胀（使用50×50正方形结构元素）
se_chip = strel('square', 50);
BW_eroded = imerode(BW_chip, se_chip);
BW_morph = imdilate(BW_eroded, se_chip);

% 3. 反转后连通域分析，保留最大连通域
BW_inverted = ~BW_morph;
CC_chip = bwconncomp(BW_inverted);
stats_chip = regionprops(CC_chip, 'Area');
[~, maxIdx_chip] = max([stats_chip.Area]);
BW_max_chip = false(size(BW_inverted));
BW_max_chip(CC_chip.PixelIdxList{maxIdx_chip}) = true;

% 4. 制作掩膜：连通域区域为1，其他为-1
mask_chip = ones(size(I_original)) * (-1);
mask_chip(BW_max_chip) = 1;

% 5. 掩膜与原始灰度图相乘
I_masked = double(I_original) .* mask_chip;
I_gray = I_original;
I_gray(mask_chip == -1) = 0; % 背景区域设为0

%% 基于粗定位结果进行Otsu分割获取基板区域（与pic5_04保持一致）
% 仅对芯片区域（掩膜区域）进行阈值分割
% 提取掩膜区域内的像素值进行大津法计算
roi_pixels = I_gray(mask_chip == 1);
level = graythresh(roi_pixels);
% level = 0.8;  % 可选：手动设置阈值
BW = imbinarize(I_gray, level);
BW_substrate = ~BW; % 基板为1(白色)

% 提取最大连通域作为基板
CC = bwconncomp(BW_substrate);
stats = regionprops(CC, 'Area');
[~, maxIdx] = max([stats.Area]);
BW_substrate_main = false(size(BW));
BW_substrate_main(CC.PixelIdxList{maxIdx}) = true;

%% 形态学闭运算处理 - 消除过曝白点和划伤
% 自适应确定结构元素半径
[img_h, img_w] = size(I_gray);
rho = min(img_h, img_w) / 1000; % 图像分辨率因子
d = 2; % 过曝区域典型尺寸(像素)
r = ceil(rho * d);
r = max(r, 3); % 至少为3

se = strel('disk', r);
BW_substrate_closed = imclose(BW_substrate_main, se);

%% 基板内高灰度区域分布
% 在处理后的基板掩膜内提取高灰度连通域（焊球候选）
BW_high = double(abs(BW_substrate_closed-1).*255); % 高灰度区域
BW_balls_all = BW_high; % 使用闭运算后的基板掩膜

% 连通域分析
CC_balls = bwconncomp(BW_balls_all);
stats_balls = regionprops(CC_balls, 'Area', 'Perimeter', 'Centroid', 'BoundingBox');

%% 圆形度与面积约束筛选
% 计算圆形度: C = 4*pi*A / P^2
circularity = zeros(length(stats_balls), 1);
for i = 1:length(stats_balls)
    A = stats_balls(i).Area;
    P = stats_balls(i).Perimeter;
    if P > 0
        circularity(i) = 4 * pi * A / (P^2);
    end
end

% 设定约束阈值
C_thr = 0.82; % 圆形度阈值
A_ball_min = 300; % 最小面积
A_ball_max = 4000; % 最大面积

% 筛选满足圆形度和面积约束的候选焊球
valid_geometry = (circularity > C_thr) & ...
                 ([stats_balls.Area]' >= A_ball_min) & ...
                 ([stats_balls.Area]' <= A_ball_max);

%% 边界距离约束剔除
% 计算基板边界距离变换（使用闭运算后的基板）
BW_substrate_boundary = bwperim(BW_substrate_closed);
D = bwdist(BW_substrate_boundary);

% 对每个候选焊球，检查其质心到边界的距离
d_edge_thr = 1; % 边界距离阈值
valid_distance = false(length(stats_balls), 1);

for i = 1:length(stats_balls)
    if valid_geometry(i)
        cx = round(stats_balls(i).Centroid(1));
        cy = round(stats_balls(i).Centroid(2));
        if cx > 0 && cx <= size(D, 2) && cy > 0 && cy <= size(D, 1)
            dist_to_edge = D(cy, cx);
            valid_distance(i) = dist_to_edge > d_edge_thr;
        end
    end
end

%% 最终焊球候选区域
valid_final = valid_geometry & valid_distance;
valid_indices = find(valid_final);

% 使用有效焊球掩膜
BW_balls = false(size(I_gray));
for i = 1:length(valid_indices)
    BW_balls(CC_balls.PixelIdxList{valid_indices(i)}) = true;
end

if isempty(valid_indices)
    error('未找到有效焊球');
end

% 选择第一个有效焊球进行演示
ball_idx = valid_indices(1);
ball_center = stats_balls(ball_idx).Centroid;
ball_area = stats_balls(ball_idx).Area;
estimated_radius = sqrt(ball_area / pi);

%% Canny边缘检测
edges = edge(double(BW_balls), 'Canny');

% 在焊球周围提取边缘点
search_radius = estimated_radius * 2;
[edge_y, edge_x] = find(edges);
distances = sqrt((edge_x - ball_center(1)).^2 + (edge_y - ball_center(2)).^2);
nearby_edges = distances < search_radius & distances > estimated_radius * 0.5;
edge_x_ball = edge_x(nearby_edges);
edge_y_ball = edge_y(nearby_edges);

if length(edge_x_ball) < 5
    warning('边缘点过少，使用全部边缘点');
    edge_x_ball = edge_x;
    edge_y_ball = edge_y;
end

%% 迭代圆拟合
max_iter = 10;
beta = 2.0; % 阈值系数

% 存储每轮迭代结果
iter_data = cell(max_iter, 1);
x_current = edge_x_ball;
y_current = edge_y_ball;

for iter = 1:max_iter
    % 圆拟合
    [xc, yc, r] = fit_circle(x_current, y_current);
    
    % 计算径向偏差
    radial_distances = sqrt((x_current - xc).^2 + (y_current - yc).^2);
    radial_errors = abs(radial_distances - r);
    
    % 动态阈值
    sigma_r = std(radial_errors);
    threshold = beta * sigma_r;
    
    % 标记离群点
    outliers = radial_errors > threshold;
    
    % 保存迭代数据
    iter_data{iter}.x = x_current;
    iter_data{iter}.y = y_current;
    iter_data{iter}.xc = xc;
    iter_data{iter}.yc = yc;
    iter_data{iter}.r = r;
    iter_data{iter}.outliers = outliers;
    iter_data{iter}.sigma = sigma_r;
    
    % 移除离群点
    if sum(~outliers) < 5 || sum(outliers) == 0
        break;
    end
    x_current = x_current(~outliers);
    y_current = y_current(~outliers);
end

num_iters = min(iter, max_iter);

%% 创建图形
fig = figure('Position', [100, 100, 1400, 800], 'Color', 'w');

% 选择显示的区域(焊球周围)
roi_size = round(estimated_radius * 3);
x_min = max(1, round(ball_center(1) - roi_size));
x_max = min(size(I_gray, 2), round(ball_center(1) + roi_size));
y_min = max(1, round(ball_center(2) - roi_size));
y_max = min(size(I_gray, 1), round(ball_center(2) + roi_size));

I_roi = I_gray(y_min:y_max, x_min:x_max);

% (a) Canny提取的原始边缘点
subplot(2, 2, 1);
imshow(I_roi, []);
hold on;
% 转换坐标到ROI
x_plot = edge_x_ball - x_min + 1;
y_plot = edge_y_ball - y_min + 1;
plot(x_plot, y_plot, '.', 'Color', [1, 0.7, 0.7], 'MarkerSize', 6);
title('(a) Canny提取的原始边缘点', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
text(5, 15, sprintf('共 %d 个边缘点', length(edge_x_ball)), ...
    'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold', ...
    'BackgroundColor', [1, 1, 1, 0.7]);

% (b) 第1轮迭代剔除虚假点
if num_iters >= 1
    subplot(2, 2, 2);
    imshow(I_roi, []);
    hold on;
    
    data1 = iter_data{1};
    x_valid = data1.x(~data1.outliers) - x_min + 1;
    y_valid = data1.y(~data1.outliers) - y_min + 1;
    x_outlier = data1.x(data1.outliers) - x_min + 1;
    y_outlier = data1.y(data1.outliers) - y_min + 1;
    
    plot(x_valid, y_valid, 'b.', 'MarkerSize', 6);
    plot(x_outlier, y_outlier, 'r.', 'MarkerSize', 8);
    
    % 绘制拟合圆
    theta = linspace(0, 2*pi, 100);
    xc_plot = data1.xc + data1.r * cos(theta) - x_min + 1;
    yc_plot = data1.yc + data1.r * sin(theta) - y_min + 1;
    plot(xc_plot, yc_plot, 'g-', 'LineWidth', 1.5);
    plot(data1.xc - x_min + 1, data1.yc - y_min + 1, 'g+', 'MarkerSize', 12, 'LineWidth', 2);
    
    title('(b) 第1轮迭代剔除虚假点', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
    legend('保留点', '剔除点', '拟合圆', 'Location', 'best');
    text(5, 15, sprintf('剔除 %d 个点', sum(data1.outliers)), ...
        'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', [1, 1, 1, 0.7]);
end

% (c) 第3轮收敛结果
iter_show = min(3, num_iters);
if num_iters >= iter_show
    subplot(2, 2, 3);
    imshow(I_roi, []);
    hold on;
    
    data_final = iter_data{iter_show};
    x_valid = data_final.x - x_min + 1;
    y_valid = data_final.y - y_min + 1;
    
    plot(x_valid, y_valid, '.', 'Color', [0, 0, 0.8], 'MarkerSize', 6);
    
    % 绘制拟合圆
    theta = linspace(0, 2*pi, 100);
    xc_plot = data_final.xc + data_final.r * cos(theta) - x_min + 1;
    yc_plot = data_final.yc + data_final.r * sin(theta) - y_min + 1;
    plot(xc_plot, yc_plot, 'g-', 'LineWidth', 2);
    plot(data_final.xc - x_min + 1, data_final.yc - y_min + 1, 'g+', ...
        'MarkerSize', 15, 'LineWidth', 3);
    
    title(sprintf('(c) 第%d轮收敛结果', iter_show), 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
    legend('有效边缘点', '拟合圆', '圆心', 'Location', 'best');
    text(5, 15, sprintf('σ=%.2f像素', data_final.sigma), ...
        'Color', 'g', 'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', [1, 1, 1, 0.7]);
end

% (d) 亚像素级圆心定位精度验证
subplot(2, 2, 4);
if num_iters >= 1
    % 绘制收敛曲线
    iters_plot = 1:num_iters;
    sigmas = zeros(num_iters, 1);
    remaining_points = zeros(num_iters, 1);
    
    for i = 1:num_iters
        sigmas(i) = iter_data{i}.sigma;
        remaining_points(i) = length(iter_data{i}.x);
    end
    
    yyaxis left;
    plot(iters_plot, sigmas, 'b-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    ylabel('径向偏差标准差 σ (像素)', 'FontSize', 11, 'FontWeight', 'bold');
    ylim([0, max(sigmas) * 1.2]);
    
    yyaxis right;
    plot(iters_plot, remaining_points, 'r-s', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    ylabel('保留边缘点数', 'FontSize', 11, 'FontWeight', 'bold');
    
    xlabel('迭代轮数', 'FontSize', 11, 'FontWeight', 'bold');
    title('(d) 亚像素级圆心定位精度验证', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
    legend('拟合误差', '边缘点数', 'Location', 'best');
    grid on;
    set(gca, 'FontSize', 10);
    
    % 添加最终精度信息
    final_xc = iter_data{num_iters}.xc;
    final_yc = iter_data{num_iters}.yc;
    final_r = iter_data{num_iters}.r;
    text(1, max(sigmas) * 1.1, sprintf('最终圆心: (%.2f, %.2f)\n半径: %.2f像素', ...
        final_xc, final_yc, final_r), ...
        'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', [1, 1, 1, 0.8]);
end

%% 添加总标题
sgtitle('图5-5 针对过曝边缘的迭代剔除圆拟合策略', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

%% 保存图片
saveas(fig, 'fig5_05_圆拟合迭代剔除.png');
% saveas(fig, 'fig5_05_圆拟合迭代剔除.fig');
fprintf('图5-5生成完成\n');

%% 对所有焊球进行高精度圆拟合
fprintf('开始对所有焊球进行亚像素级圆拟合...\n');
num_balls = length(valid_indices);
ballCircles = struct('xc', {}, 'yc', {}, 'r', {}, 'ballIdx', {});

for idx = 1:num_balls
    ball_idx_current = valid_indices(idx);
    ball_center_current = stats_balls(ball_idx_current).Centroid;
    ball_area_current = stats_balls(ball_idx_current).Area;
    estimated_radius_current = sqrt(ball_area_current / pi);
    
    % Canny边缘检测提取该焊球的边缘点
    search_radius_current = estimated_radius_current * 2;
    distances_current = sqrt((edge_x - ball_center_current(1)).^2 + (edge_y - ball_center_current(2)).^2);
    nearby_edges_current = distances_current < search_radius_current & distances_current > estimated_radius_current * 0.5;
    edge_x_ball_current = edge_x(nearby_edges_current);
    edge_y_ball_current = edge_y(nearby_edges_current);
    
    if length(edge_x_ball_current) < 5
        % 边缘点过少，使用初始质心和估计半径
        ballCircles(idx).xc = ball_center_current(1);
        ballCircles(idx).yc = ball_center_current(2);
        ballCircles(idx).r = estimated_radius_current;
        ballCircles(idx).ballIdx = ball_idx_current;
        continue;
    end
    
    % 迭代圆拟合
    max_iter_current = 50;
    beta_current = 2.0;
    x_current = edge_x_ball_current;
    y_current = edge_y_ball_current;
    
    xc_final = ball_center_current(1);
    yc_final = ball_center_current(2);
    r_final = estimated_radius_current;
    
    for iter = 1:max_iter_current
        [xc_tmp, yc_tmp, r_tmp] = fit_circle(x_current, y_current);
        radial_distances = sqrt((x_current - xc_tmp).^2 + (y_current - yc_tmp).^2);
        radial_errors = abs(radial_distances - r_tmp);
        sigma_r = std(radial_errors);
        threshold_current = beta_current * sigma_r;
        outliers = radial_errors > threshold_current;
        
        if sum(~outliers) < 5 || sum(outliers) == 0
            xc_final = xc_tmp;
            yc_final = yc_tmp;
            r_final = r_tmp;
            break;
        end
        
        x_current = x_current(~outliers);
        y_current = y_current(~outliers);
        xc_final = xc_tmp;
        yc_final = yc_tmp;
        r_final = r_tmp;
    end
    
    ballCircles(idx).xc = xc_final;
    ballCircles(idx).yc = yc_final;
    ballCircles(idx).r = r_final;
    ballCircles(idx).ballIdx = ball_idx_current;
end

fprintf('共完成 %d 个焊球的圆拟合\n', length(ballCircles));

%% 保存焊球掩膜和圆心数据
hanqiuMask = BW_balls;  % 焊球区域掩膜
save('hanqiuMask.mat', 'hanqiuMask');
save('ballCircles.mat', 'ballCircles');
mask_chip = (mask_chip+1)/2;
save('I_masked.mat', 'mask_chip');
fprintf('已保存 hanqiuMask.mat 和 ballCircles.mat\n');

%% 芯片掩膜旋转矫正
fprintf('\n开始芯片旋转矫正...\n');

% 1. 提取芯片掩膜的边缘并拟合矩形
BW_chip_mask = (mask_chip == 1);

% 提取边缘点
BW_edge = edge(BW_chip_mask, 'Canny');
[edge_y, edge_x] = find(BW_edge);

if length(edge_x) < 100
    warning('边缘点过少，使用区域属性方法');
    stats_rect = regionprops(BW_chip_mask, 'Orientation', 'Centroid');
    if ~isempty(stats_rect)
        rotation_angle = -stats_rect(1).Orientation;
        chip_centroid = stats_rect(1).Centroid;
    else
        rotation_angle = 0;
        chip_centroid = [size(BW_chip_mask, 2)/2, size(BW_chip_mask, 1)/2];
    end
else
    % 2. 使用PCA对边缘点进行主成分分析，找到矩形的主方向
    edge_points = [edge_x, edge_y];
    edge_points_centered = edge_points - mean(edge_points, 1);
    
    % PCA分析
    [coeff, ~, ~] = pca(edge_points_centered);
    
    % 第一主成分方向（最长边方向）
    principal_direction = coeff(:, 1);
    
    % 计算主方向与X轴的夹角
    main_direction_angle = atan2d(principal_direction(2), principal_direction(1));
    
    % 计算旋转角度（使主方向水平）- 需要旋转负的主方向角度
    rotation_angle = -main_direction_angle;
    
    % 矫正到[-45, 45]度范围（使矩形最接近水平）
    if rotation_angle > 45
        rotation_angle = rotation_angle - 90;
    elseif rotation_angle < -45
        rotation_angle = rotation_angle + 90;
    end
    
    % 计算芯片质心
    chip_centroid = mean(edge_points, 1);
    
    % 计算拟合矩形的顶点（基于凸包和PCA结果，更贴合边缘）
    % 1. 先计算边缘点的凸包，去除内部点
    try
        hull_indices = convhull(edge_points(:,1), edge_points(:,2));
        hull_points = edge_points(hull_indices, :);
    catch
        % 如果凸包计算失败，使用全部边缘点
        hull_points = edge_points;
    end
    
    % 2. 在凸包点上计算投影（更精确的边界）
    hull_centered = hull_points - chip_centroid;
    proj_main = hull_centered * principal_direction;  % 在主方向上的投影
    proj_sec = hull_centered * coeff(:, 2);  % 在次方向上的投影
    
    % 3. 使用凸包点的投影极值来确定矩形尺寸（严格贴合外轮廓）
    half_length = (max(proj_main) - min(proj_main)) / 2;
    half_width = (max(proj_sec) - min(proj_sec)) / 2;
    center_offset_main = (max(proj_main) + min(proj_main)) / 2;
    center_offset_sec = (max(proj_sec) + min(proj_sec)) / 2;
    
    % 4. 矩形中心（考虑投影偏移）
    rect_center = chip_centroid + center_offset_main * principal_direction' + center_offset_sec * coeff(:, 2)';
    
    % 5. 构建矩形的四个顶点（在主成分坐标系中）
    rect_corners_local = [
        -half_length, -half_width;
        half_length, -half_width;
        half_length, half_width;
        -half_length, half_width;
        -half_length, -half_width  % 闭合矩形
    ];
    
    % 6. 转换到图像坐标系
    rect_coords = rect_corners_local * [principal_direction'; coeff(:, 2)'] + rect_center;
    
    fprintf('边缘点数量: %d\n', length(edge_x));
    fprintf('主方向角度: %.2f 度\n', main_direction_angle);
    fprintf('需要旋转角度: %.2f 度\n', rotation_angle);
end

fprintf('芯片旋转角度: %.2f 度\n', rotation_angle);
fprintf('芯片质心: (%.2f, %.2f)\n', chip_centroid(1), chip_centroid(2));

% 3. 构建旋转矩阵（绕图像中心旋转，与imrotate保持一致）
theta = deg2rad(rotation_angle);
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

% 计算图像中心（imrotate的默认旋转中心）
img_center = [size(I_original, 2)/2, size(I_original, 1)/2];

% 4. 对所有焊球圆心应用旋转变换（绕图像中心）
num_balls_total = length(ballCircles);
centers_original = zeros(num_balls_total, 2);
centers_rotated = zeros(num_balls_total, 2);

for idx = 1:num_balls_total
    centers_original(idx, :) = [ballCircles(idx).xc, ballCircles(idx).yc];
    
    % 平移到图像中心，旋转，再平移回去（与imrotate保持一致）
    centered = centers_original(idx, :) - img_center;
    rotated = (R * centered')';
    centers_rotated(idx, :) = rotated + img_center;
end

% 旋转整个图像用于可视化（绕图像中心，使用负角度以匹配点的旋转方向）
I_rotated = imrotate(I_original, -rotation_angle, 'bilinear', 'crop');

% 5. 在旋转后的坐标系中进行排序（从左上到右下，逐行）
% 先按Y坐标聚类成行，再在每行内按X坐标排序

Y_coords = centers_rotated(:, 2);
X_coords = centers_rotated(:, 1);

% 改进的行分组算法：自适应跳跃检测
[Y_sorted_vals, Y_sorted_idx] = sort(Y_coords);
Y_diffs = diff(Y_sorted_vals);

% 计算差值的统计特征
if length(Y_diffs) >= 3
    % 使用四分位数识别显著跳跃
    Q1 = quantile(Y_diffs, 0.25);
    Q3 = quantile(Y_diffs, 0.75);
    IQR = Q3 - Q1;
    
    % 自适应阈值：超过Q3 + 1.5*IQR的被认为是行间距
    % 这个阈值能更好地处理不规整的分布
    if IQR > 0
        row_threshold = Q3 + 1.5 * IQR;
    else
        % 如果IQR为0（所有差值非常接近），使用更保守的策略
        row_threshold = mean(Y_diffs) + 2 * std(Y_diffs);
    end
    
    % 确保阈值至少是中位数的1.5倍（避免过度分割）
    row_threshold = max(row_threshold, median(Y_diffs) * 1.5);
    
    fprintf('Y坐标差值统计: Q1=%.2f, Q3=%.2f, IQR=%.2f\n', Q1, Q3, IQR);
    fprintf('自适应阈值: %.2f 像素\n', row_threshold);
else
    % 焊球太少，使用简单策略
    row_threshold = std(Y_coords) * 0.5;
    fprintf('焊球数量较少，使用标准差策略，阈值: %.2f 像素\n', row_threshold);
end

% 分组：从上到下扫描，识别行边界
row_assignments = zeros(num_balls_total, 1);
current_row = 1;
row_assignments(Y_sorted_idx(1)) = current_row;

for i = 2:num_balls_total
    if Y_diffs(i-1) > row_threshold
        current_row = current_row + 1;
        fprintf('  检测到新行: 在索引%d处，Y差值=%.2f > 阈值%.2f\n', ...
                i, Y_diffs(i-1), row_threshold);
    end
    row_assignments(Y_sorted_idx(i)) = current_row;
end

num_rows = max(row_assignments);

% 输出每行的焊球数量统计
fprintf('检测到 %d 行焊球，每行数量: ', num_rows);
for row = 1:num_rows
    row_count = sum(row_assignments == row);
    fprintf('%d ', row_count);
end
fprintf('\n');

% 对每一行内部按X坐标排序，并分配全局编号
ball_order = zeros(num_balls_total, 1);
global_index = 1;

for row = 1:num_rows
    row_balls = find(row_assignments == row);
    [~, sort_idx] = sort(X_coords(row_balls));
    sorted_row_balls = row_balls(sort_idx);
    
    for j = 1:length(sorted_row_balls)
        ball_order(sorted_row_balls(j)) = global_index;
        global_index = global_index + 1;
    end
end

% 保存排序信息到ballCircles结构
for idx = 1:num_balls_total
    ballCircles(idx).order = ball_order(idx);
    ballCircles(idx).row = row_assignments(idx);
end

fprintf('焊球排序完成\n');

% 重新保存ballCircles（包含排序信息）
save('ballCircles.mat', 'ballCircles');
fprintf('已更新 ballCircles.mat（包含排序信息）\n');

%% 绘制圆心排序结果
fig3 = figure('Position', [100, 100, 1600, 900], 'Color', 'w');

% 子图1: 芯片边缘拟合与主方向
subplot(2, 3, 1);
imshow(BW_chip_mask);
hold on;
if exist('edge_points', 'var') && exist('principal_direction', 'var')
    % 绘制边缘点
    plot(edge_x, edge_y, 'r.', 'MarkerSize', 2);
    
    % 绘制已计算好的拟合矩形
    if exist('rect_coords', 'var')
        plot(rect_coords(:, 1), rect_coords(:, 2), 'm-', 'LineWidth', 2.5);
    end
    
    % 绘制主方向向量
    scale = 200; % 向量长度
    quiver(chip_centroid(1), chip_centroid(2), ...
           principal_direction(1)*scale, principal_direction(2)*scale, ...
           0, 'g', 'LineWidth', 3, 'MaxHeadSize', 2);
    
    % 绘制次方向向量（垂直于主方向）
    secondary_direction = coeff(:, 2);
    quiver(chip_centroid(1), chip_centroid(2), ...
           secondary_direction(1)*scale, secondary_direction(2)*scale, ...
           0, 'b', 'LineWidth', 3, 'MaxHeadSize', 2);
    
    % 绘制质心
    plot(chip_centroid(1), chip_centroid(2), 'y+', 'MarkerSize', 20, 'LineWidth', 3);
    
    % 标注旋转方向和角度（方向相反）
    if rotation_angle > 0
        rotation_text = sprintf('↻ %.1f°', rotation_angle);  % 正角度实际是顺时针
    else
        rotation_text = sprintf('↺ %.1f°', abs(rotation_angle));  % 负角度实际是逆时针
    end
    text(chip_centroid(1) + 50, chip_centroid(2) - 50, rotation_text, ...
        'Color', 'c', 'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', [0, 0, 0, 0.6]);
    
    legend('边缘点', '拟合矩形', '主方向', '次方向', '质心', 'Location', 'best');
end
title('(a) 芯片边缘拟合与主方向', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

% 子图2: 原图标记焊球圆心（无编号）
subplot(2, 3, 2);
imshow(I_original);
hold on;
for idx = 1:length(ballCircles)
    xc = ballCircles(idx).xc;
    yc = ballCircles(idx).yc;
    plot(xc, yc, 'r+', 'MarkerSize', 10, 'LineWidth', 2);
end
title('(b) 原图标记焊球圆心', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

% 子图3: 旋转后的图像与圆心位置
subplot(2, 3, 3);
imshow(I_rotated);
hold on;

% 使用不同颜色标记不同行
cmap = lines(num_rows);
for idx = 1:length(ballCircles)
    % 使用旋转后的坐标
    xc_rot = centers_rotated(idx, 1);
    yc_rot = centers_rotated(idx, 2);
    order_num = ballCircles(idx).order;
    row_num = ballCircles(idx).row;
    
    % 根据行数选择颜色
    color = cmap(row_num, :);
    
    % 绘制圆心
    plot(xc_rot, yc_rot, 'o', 'MarkerSize', 8, 'LineWidth', 2, ...
        'MarkerEdgeColor', color, 'MarkerFaceColor', color);
    
    % 标注编号（白色背景）
    text(xc_rot, yc_rot, sprintf('%d', order_num), ...
        'Color', 'k', 'FontSize', 7, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'BackgroundColor', [1, 1, 1, 0.7]);
end
title(sprintf('(c) 旋转后图像与排序（共%d行）', num_rows), ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

% 子图4: 原图标记排序编号
subplot(2, 3, 4);
imshow(I_original);
hold on;

% 绘制所有焊球圆心和排序编号
for idx = 1:length(ballCircles)
    xc = ballCircles(idx).xc;
    yc = ballCircles(idx).yc;
    r = ballCircles(idx).r;
    order_num = ballCircles(idx).order;
    
    % 绘制圆
    theta_circle = linspace(0, 2*pi, 100);
    x_circle = xc + r * cos(theta_circle);
    y_circle = yc + r * sin(theta_circle);
    plot(x_circle, y_circle, 'g-', 'LineWidth', 1);
    
    % 绘制圆心
    plot(xc, yc, 'r+', 'MarkerSize', 10, 'LineWidth', 2);
    
    % 标注编号
    text(xc, yc-r*1.5, sprintf('%d', order_num), ...
        'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', ...
        'BackgroundColor', [1, 1, 1, 0.8]);
end

title('(d) 原图标记排序编号', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

% 子图5: 旋转后坐标分布（散点图）
subplot(2, 3, 5);
for row = 1:num_rows
    row_balls = find([ballCircles.row] == row);
    x_row = centers_rotated(row_balls, 1);
    y_row = centers_rotated(row_balls, 2);
    plot(x_row, y_row, 'o-', 'LineWidth', 1.5, 'MarkerSize', 8);
    hold on;
end
xlabel('X (旋转后)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Y (旋转后)', 'FontSize', 10, 'FontWeight', 'bold');
title('(e) 旋转后坐标分布', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;
axis equal;
hold off;

% 子图6: 每行焊球数量统计
subplot(2, 3, 6);
balls_per_row = zeros(num_rows, 1);
for row = 1:num_rows
    balls_per_row(row) = sum([ballCircles.row] == row);
end
bar(1:num_rows, balls_per_row, 'FaceColor', [0.2, 0.6, 0.8], 'EdgeColor', 'k');
xlabel('行号', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('焊球数量', 'FontSize', 10, 'FontWeight', 'bold');
title('(f) 每行焊球数量统计', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;
for row = 1:num_rows
    text(row, balls_per_row(row), sprintf('%d', balls_per_row(row)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontWeight', 'bold', 'FontSize', 9);
end

% 添加总标题
sgtitle(sprintf('焊球圆心排序结果 (旋转角度: %.2f°)', rotation_angle), ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 保存图片
saveas(fig3, 'fig5_07_焊球圆心排序.png');
fprintf('已生成焊球圆心排序结果图\n');

%% 在原图上展示所有拟合圆和圆心
fig2 = figure('Position', [100, 100, 1200, 900], 'Color', 'w');

% 子图1: 原图叠加所有拟合圆和圆心
subplot(2, 2, 1);
imshow(I_original);
hold on;
theta = linspace(0, 2*pi, 100);
for idx = 1:length(ballCircles)
    xc = ballCircles(idx).xc;
    yc = ballCircles(idx).yc;
    r = ballCircles(idx).r;
    
    % 绘制拟合圆
    x_circle = xc + r * cos(theta);
    y_circle = yc + r * sin(theta);
    plot(x_circle, y_circle, 'g-', 'LineWidth', 1.5);
    
    % 绘制圆心
    plot(xc, yc, 'r+', 'MarkerSize', 8, 'LineWidth', 2);
end
title(sprintf('(a) 所有焊球拟合圆 (共%d个)', length(ballCircles)), ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

% 子图2: 原图叠加圆心点
subplot(2, 2, 2);
imshow(I_original);
hold on;
for idx = 1:length(ballCircles)
    xc = ballCircles(idx).xc;
    yc = ballCircles(idx).yc;
    plot(xc, yc, 'r.', 'MarkerSize', 15);
end
title('(b) 所有焊球圆心位置', ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

% 子图3: 去除杂点后的边缘点（焊球区域）
subplot(2, 2, 3);
imshow(I_original);
hold on;
% 显示最终保留的焊球区域
B_balls = bwboundaries(BW_balls);
for k = 1:length(B_balls)
    boundary = B_balls{k};
    plot(boundary(:,2), boundary(:,1), 'g-', 'LineWidth', 1);
end
title('(c) 去除杂点后的焊球区域', ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

% 子图4: 半径分布统计
subplot(2, 2, 4);
radii = [ballCircles.r];
histogram(radii, 20, 'FaceColor', [0.2, 0.6, 0.8], 'EdgeColor', 'k');
xlabel('半径 (像素)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('数量', 'FontSize', 11, 'FontWeight', 'bold');
title('(d) 焊球半径分布统计', ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;
% 添加统计信息
mean_r = mean(radii);
std_r = std(radii);
text(0.6, 0.9, sprintf('平均半径: %.2f\n标准差: %.2f', mean_r, std_r), ...
    'Units', 'normalized', 'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [1, 1, 1, 0.8]);

% 添加总标题
sgtitle('图5-6 所有焊球的圆拟合结果展示', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 保存图片
saveas(fig2, 'fig5_06_所有焊球圆拟合结果.png');
fprintf('已生成所有焊球圆拟合结果展示图\n');

%% 辅助函数：圆拟合(最小二乘法)
function [xc, yc, r] = fit_circle(x, y)
    % 使用代数拟合方法
    n = length(x);
    if n < 3
        xc = mean(x);
        yc = mean(y);
        r = 0;
        return;
    end
    
    % 构建矩阵
    A = [2*x, 2*y, ones(n, 1)];
    b = x.^2 + y.^2;
    
    % 最小二乘求解
    params = A \ b;
    
    xc = params(1);
    yc = params(2);
    r = sqrt(params(3) + xc^2 + yc^2);
end
