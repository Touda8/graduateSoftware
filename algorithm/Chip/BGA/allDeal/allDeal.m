% allDeal.m - 焊球检测、测量与数据导出整合脚本
% 
% 功能：
%   1. 基于pic5_05：焊球分割、圆拟合、编号排序
%   2. 基于pic5_08：3D曲面拟合、高度测量
%   3. 按照pic5_05编号顺序输出Excel测量结果
%
% 输出文件：
%   - 焊球高度测量结果.xlsx (Excel测量数据)
%   - 焊球分割与排序结果.png (关键图片1)
%   - 多焊球顶点分布与高度.png (关键图片2)
%   - 焊球高度测量结果.png (统计图表)

clear; clc; close all;

%% ========== 输入配置 ==========
% 输入文件路径
INPUT_IMAGE = '../segment/bga.bmp';        % 2D图像
INPUT_X = '../pointCal/X.mat';             % X坐标
INPUT_Y = '../pointCal/Y.mat';             % Y坐标  
INPUT_Z = '../pointCal/Z.mat';             % Z坐标

% 输出文件路径
OUTPUT_EXCEL = '焊球高度测量结果.xlsx';
OUTPUT_FIG1 = '焊球分割与排序结果.png';
OUTPUT_FIG2 = '多焊球顶点分布与高度.png';
OUTPUT_FIG3 = '焊球高度测量结果.png';

% 算法参数配置
CONFIG = struct();
% 焊球检测参数
CONFIG.circularity_threshold = 0.85;      % 圆形度阈值
CONFIG.area_min = 300;                     % 最小面积
CONFIG.area_max = 4000;                    % 最大面积
CONFIG.edge_distance_threshold = 1;        % 边界距离阈值
CONFIG.circle_fit_iterations = 10;         % 圆拟合迭代次数
CONFIG.circle_fit_beta = 2.0;              % 圆拟合阈值系数

% 3D测量参数
CONFIG.substrate_downsample_ratio = 0.5;   % 基板降采样比例
CONFIG.substrate_angle_threshold = 30;     % 基板法向量角度阈值(度)
CONFIG.ball_angle_threshold = 45;          % 焊球法向量角度阈值(度)
CONFIG.surface_fit_iterations = 100;       % 曲面拟合迭代次数
CONFIG.surface_fit_sigma = 2.5;            % 曲面拟合sigma系数
CONFIG.z_flip = 160;                       % Z轴翻转值

fprintf('=================================================\n');
fprintf('       焊球检测、测量与数据导出整合脚本\n');
fprintf('=================================================\n\n');

%% ========== 第一部分：基于pic5_05的焊球检测与编号 ==========
fprintf('【第一部分】焊球检测、圆拟合与编号...\n');

%% 1.1 读取2D图像
fprintf('  1.1 读取原始图像...\n');
I_original = imread(INPUT_IMAGE);
if size(I_original, 3) == 3
    I_original = rgb2gray(I_original);
end

%% 1.2 芯片区域粗定位
fprintf('  1.2 芯片区域粗定位...\n');
% Otsu全局二值分割
level_chip = graythresh(I_original);
BW_chip = imbinarize(I_original, level_chip);

% 形态学操作：先腐蚀后膨胀（50×50正方形结构元素）
se_chip = strel('square', 50);
BW_eroded = imerode(BW_chip, se_chip);
BW_morph = imdilate(BW_eroded, se_chip);

% 反转后连通域分析，保留最大连通域
BW_inverted = ~BW_morph;
CC_chip = bwconncomp(BW_inverted);
stats_chip = regionprops(CC_chip, 'Area');
[~, maxIdx_chip] = max([stats_chip.Area]);
BW_max_chip = false(size(BW_inverted));
BW_max_chip(CC_chip.PixelIdxList{maxIdx_chip}) = true;

% 制作掩膜
mask_chip = ones(size(I_original)) * (-1);
mask_chip(BW_max_chip) = 1;

% 掩膜处理
I_gray = I_original;
I_gray(mask_chip == -1) = 0;

%% 1.3 基板区域分割
fprintf('  1.3 基板区域分割...\n');
roi_pixels = I_gray(mask_chip == 1);
level = graythresh(roi_pixels);
BW = imbinarize(I_gray, level);
BW_substrate = ~BW;

% 提取最大连通域作为基板
CC = bwconncomp(BW_substrate);
stats = regionprops(CC, 'Area');
[~, maxIdx] = max([stats.Area]);
BW_substrate_main = false(size(BW));
BW_substrate_main(CC.PixelIdxList{maxIdx}) = true;

%% 1.4 形态学闭运算 - 消除过曝白点
fprintf('  1.4 形态学闭运算处理...\n');
[img_h, img_w] = size(I_gray);
rho = min(img_h, img_w) / 1000;
d = 2;
r = ceil(rho * d);
r = max(r, 3);

se = strel('disk', r);
BW_substrate_closed = imclose(BW_substrate_main, se);

%% 1.5 焊球候选区域检测
fprintf('  1.5 焊球候选区域检测...\n');
BW_high = double(abs(BW_substrate_closed-1).*255);
BW_balls_all = BW_high;

% 连通域分析
CC_balls = bwconncomp(BW_balls_all);
stats_balls = regionprops(CC_balls, 'Area', 'Perimeter', 'Centroid', 'BoundingBox');

%% 1.6 圆形度与面积约束筛选
fprintf('  1.6 圆形度与面积约束筛选...\n');
circularity = zeros(length(stats_balls), 1);
for i = 1:length(stats_balls)
    A = stats_balls(i).Area;
    P = stats_balls(i).Perimeter;
    if P > 0
        circularity(i) = 4 * pi * A / (P^2);
    end
end

C_thr = CONFIG.circularity_threshold;
A_ball_min = CONFIG.area_min;
A_ball_max = CONFIG.area_max;

valid_geometry = (circularity > C_thr) & ...
                 ([stats_balls.Area]' >= A_ball_min) & ...
                 ([stats_balls.Area]' <= A_ball_max);

%% 1.7 边界距离约束
fprintf('  1.7 边界距离约束剔除...\n');
BW_substrate_boundary = bwperim(BW_substrate_closed);
D = bwdist(BW_substrate_boundary);

d_edge_thr = CONFIG.edge_distance_threshold;
valid_distance = false(length(stats_balls), 1);

for i = 1:length(stats_balls)
    if valid_geometry(i)
        centroid = stats_balls(i).Centroid;
        x_c = round(centroid(1));
        y_c = round(centroid(2));
        if x_c >= 1 && x_c <= size(D, 2) && y_c >= 1 && y_c <= size(D, 1)
            if D(y_c, x_c) > d_edge_thr
                valid_distance(i) = true;
            end
        end
    end
end

%% 1.8 生成最终焊球掩膜
fprintf('  1.8 生成最终焊球掩膜...\n');
valid_final = valid_geometry & valid_distance;
valid_indices = find(valid_final);

BW_balls = false(size(I_gray));
for i = 1:length(valid_indices)
    BW_balls(CC_balls.PixelIdxList{valid_indices(i)}) = true;
end

if isempty(valid_indices)
    error('未找到有效焊球');
end

fprintf('  检测到 %d 个有效焊球\n', length(valid_indices));

%% 1.9 对所有焊球进行高精度圆拟合
fprintf('  1.9 对所有焊球进行高精度圆拟合...\n');
num_balls = length(valid_indices);
ballCircles = struct('xc', {}, 'yc', {}, 'r', {}, 'ballIdx', {});

edges = edge(double(BW_balls), 'Canny');
[edge_y, edge_x] = find(edges);

for idx = 1:num_balls
    ball_idx_current = valid_indices(idx);
    ball_center = stats_balls(ball_idx_current).Centroid;
    ball_area = stats_balls(ball_idx_current).Area;
    estimated_radius = sqrt(ball_area / pi);
    
    % 提取焊球周围的边缘点
    search_radius = estimated_radius * 2;
    distances = sqrt((edge_x - ball_center(1)).^2 + (edge_y - ball_center(2)).^2);
    nearby_edges = distances < search_radius & distances > estimated_radius * 0.5;
    edge_x_ball = edge_x(nearby_edges);
    edge_y_ball = edge_y(nearby_edges);
    
    if length(edge_x_ball) < 5
        continue;
    end
    
    % 迭代圆拟合
    max_iter = CONFIG.circle_fit_iterations;
    beta = CONFIG.circle_fit_beta;
    x_current = edge_x_ball;
    y_current = edge_y_ball;
    
    for iter = 1:max_iter
        [xc, yc, r] = fit_circle(x_current, y_current);
        
        radial_distances = sqrt((x_current - xc).^2 + (y_current - yc).^2);
        radial_errors = abs(radial_distances - r);
        
        sigma_r = std(radial_errors);
        threshold = beta * sigma_r;
        
        outliers = radial_errors > threshold;
        
        if sum(~outliers) < 5 || sum(outliers) == 0
            break;
        end
        x_current = x_current(~outliers);
        y_current = y_current(~outliers);
    end
    
    % 保存拟合结果
    ballCircles(idx).xc = xc;
    ballCircles(idx).yc = yc;
    ballCircles(idx).r = r;
    ballCircles(idx).ballIdx = ball_idx_current;
end

fprintf('  完成 %d 个焊球的圆拟合\n', length(ballCircles));

%% 1.10 芯片旋转矫正与焊球排序
fprintf('  1.10 芯片旋转矫正与焊球排序...\n');

% 提取芯片掩膜边缘
BW_chip_mask = (mask_chip == 1);
BW_edge = edge(BW_chip_mask, 'Canny');
[edge_y, edge_x] = find(BW_edge);

if length(edge_x) >= 100
    % PCA主方向分析（与pic5_05.m保持一致）
    edge_points = [edge_x, edge_y];
    edge_points_centered = edge_points - mean(edge_points, 1);
    
    % 使用PCA函数
    [coeff, ~, ~] = pca(edge_points_centered);
    
    % 第一主成分方向（最长边方向）
    principal_direction = coeff(:, 1);
    
    % 计算主方向与X轴的夹角
    main_direction_angle = atan2d(principal_direction(2), principal_direction(1));
    
    % 计算旋转角度（使主方向水平）- 需要旋转负的主方向角度
    rotation_angle = -main_direction_angle;
    
    % 调整到[-45, 45]范围
    if rotation_angle > 45
        rotation_angle = rotation_angle - 90;
    elseif rotation_angle < -45
        rotation_angle = rotation_angle + 90;
    end
    
    % 计算芯片质心
    chip_centroid = mean(edge_points, 1);
else
    rotation_angle = 0;
    chip_centroid = [size(I_original, 2)/2, size(I_original, 1)/2];
end

fprintf('  旋转角度: %.2f 度\n', rotation_angle);

% 旋转变换
theta = deg2rad(rotation_angle);
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
img_center = [size(I_original, 2)/2, size(I_original, 1)/2];

% 对所有焊球圆心应用旋转
num_balls_total = length(ballCircles);
centers_rotated = zeros(num_balls_total, 2);

for idx = 1:num_balls_total
    center_original = [ballCircles(idx).xc, ballCircles(idx).yc];
    centered = center_original - img_center;
    rotated = (R * centered')';
    centers_rotated(idx, :) = rotated + img_center;
end

% 按行列排序
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
    
    fprintf('  Y坐标差值统计: Q1=%.2f, Q3=%.2f, IQR=%.2f\n', Q1, Q3, IQR);
    fprintf('  自适应阈值: %.2f 像素\n', row_threshold);
else
    % 焊球太少，使用简单策略
    row_threshold = std(Y_coords) * 0.5;
    fprintf('  焊球数量较少，使用标准差策略，阈值: %.2f 像素\n', row_threshold);
end

% 分组：从上到下扫描，识别行边界
row_assignments = zeros(num_balls_total, 1);
current_row = 1;
row_assignments(Y_sorted_idx(1)) = current_row;

for i = 2:num_balls_total
    if Y_diffs(i-1) > row_threshold
        current_row = current_row + 1;
        fprintf('    检测到新行: 在索引%d处，Y差值=%.2f > 阈值%.2f\n', ...
                i, Y_diffs(i-1), row_threshold);
    end
    row_assignments(Y_sorted_idx(i)) = current_row;
end

num_rows = max(row_assignments);

% 输出每行的焊球数量统计
fprintf('  检测到 %d 行焊球，每行数量: ', num_rows);
for row = 1:num_rows
    row_count = sum(row_assignments == row);
    fprintf('%d ', row_count);
end
fprintf('\n');

% 每行内按X坐标排序
ball_order = zeros(num_balls_total, 1);
global_index = 1;

for row = 1:num_rows
    row_indices = find(row_assignments == row);
    [~, sort_idx] = sort(X_coords(row_indices));
    sorted_indices = row_indices(sort_idx);
    
    for j = 1:length(sorted_indices)
        ball_order(sorted_indices(j)) = global_index;
        global_index = global_index + 1;
    end
end

% 保存排序信息
for idx = 1:num_balls_total
    ballCircles(idx).order = ball_order(idx);
    ballCircles(idx).row = row_assignments(idx);
end

fprintf('  焊球排序完成\n');

%% 1.11 生成并保存焊球分割与排序结果图
fprintf('  1.11 生成焊球分割与排序结果图...\n');
fig1 = figure('Position', [100, 100, 1600, 800], 'Color', 'w');

% 左图：焊球分割结果（标记圆心和圆）
subplot(1, 2, 1);
imshow(I_original);
hold on;

% 绘制所有拟合圆
theta_circle = linspace(0, 2*pi, 100);
for idx = 1:length(ballCircles)
    xc = ballCircles(idx).xc;
    yc = ballCircles(idx).yc;
    r = ballCircles(idx).r;
    
    x_circle = xc + r * cos(theta_circle);
    y_circle = yc + r * sin(theta_circle);
    plot(x_circle, y_circle, 'g-', 'LineWidth', 1.5);
    plot(xc, yc, 'r+', 'MarkerSize', 8, 'LineWidth', 2);
end

title(sprintf('(a) 焊球分割结果 (共%d个)', length(ballCircles)), ...
    'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

% 右图：排序编号标注
subplot(1, 2, 2);
imshow(I_original);
hold on;

% 使用不同颜色标记不同行
cmap = lines(num_rows);
for idx = 1:length(ballCircles)
    xc = ballCircles(idx).xc;
    yc = ballCircles(idx).yc;
    order = ballCircles(idx).order;
    row = ballCircles(idx).row;
    
    % 绘制圆心
    plot(xc, yc, 'o', 'MarkerSize', 8, 'MarkerFaceColor', cmap(row,:), ...
         'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    
    % 标注编号
    text(xc+5, yc-5, sprintf('%d', order), 'Color', 'r', ...
         'FontSize', 9, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
end

title(sprintf('(b) 焊球排序编号 (共%d行)', num_rows), ...
    'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimHei');
hold off;

sgtitle('焊球分割与排序结果', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimHei');

saveas(fig1, OUTPUT_FIG1);
fprintf('  ✓ 已保存: %s\n', OUTPUT_FIG1);

fprintf('  【第一部分完成】\n\n');

%% ========== 第二部分：基于pic5_08的3D测量 ==========
fprintf('【第二部分】3D曲面拟合与高度测量...\n');

%% 2.1 加载3D坐标数据
fprintf('  2.1 加载3D坐标数据...\n');
try
    load(INPUT_X);
    load(INPUT_Y);
    load(INPUT_Z);
catch
    error('无法加载3D坐标文件');
end

Z = CONFIG.z_flip - Z;  % Z轴反转

%% 2.2 准备掩膜数据
fprintf('  2.2 准备掩膜数据...\n');
hanqiuMask = logical(BW_balls);
mask_chip_save = (mask_chip+1)/2;
I_masked = logical(mask_chip_save);

se_erode = strel('disk', 2);
hanqiuMask_eroded = imerode(hanqiuMask, se_erode);

se_dilate = strel('disk', 5);
hanqiuMask_dilated = imdilate(hanqiuMask, se_dilate);

%% 2.3 基板曲面拟合
fprintf('  2.3 基板曲面拟合（二阶多项式）...\n');
validDataMask = (Z ~= 0) & ~isnan(Z);
substrateMask = I_masked & ~hanqiuMask_dilated & validDataMask;
idx_sub = find(substrateMask);

pts_sub_init = [X(idx_sub), Y(idx_sub), Z(idx_sub)];
pts_sub_init = pts_sub_init(~any(isnan(pts_sub_init),2), :);

fprintf('  基板点云数量: %d\n', size(pts_sub_init, 1));

% 初步平面拟合（法向量）
M_plane = [pts_sub_init(:,1), pts_sub_init(:,2), ones(size(pts_sub_init,1),1)];
B_plane = M_plane \ pts_sub_init(:,3);
A_ref = -B_plane(1);
B_ref = -B_plane(2);
n_ref = [A_ref, B_ref, -1];
n_ref_normalized = n_ref / norm(n_ref);
if n_ref_normalized(3) > 0, n_ref_normalized = -n_ref_normalized; end

% 基板点云降采样
downsample_ratio = 0.5;
num_points_downsample = max(1000, round(size(pts_sub_init, 1) * downsample_ratio));
if size(pts_sub_init, 1) > num_points_downsample
    rand_idx = randperm(size(pts_sub_init, 1), num_points_downsample);
    pts_sub_downsample = pts_sub_init(rand_idx, :);
else
    pts_sub_downsample = pts_sub_init;
end

% 法向量筛选
k_neighbors_sub = 30;
KDTree_sub = KDTreeSearcher(pts_sub_downsample);
normals_sub = zeros(size(pts_sub_downsample, 1), 3);

for i = 1:size(pts_sub_downsample, 1)
    [idx_nbr, ~] = knnsearch(KDTree_sub, pts_sub_downsample(i,:), 'K', min(k_neighbors_sub, size(pts_sub_downsample,1)));
    nbr_points = pts_sub_downsample(idx_nbr, :);
    coeff = pca(nbr_points);
    normal = coeff(:, 3)';
    if dot(normal, n_ref_normalized) < 0, normal = -normal; end
    normals_sub(i, :) = normal;
end

dot_prods_sub = normals_sub * n_ref_normalized';
dot_prods_sub = max(min(dot_prods_sub, 1), 0);
thetas_sub = acos(dot_prods_sub) * 180 / pi;
theta_thr_sub = 30;
valid_sub_indices = thetas_sub <= theta_thr_sub;
pts_sub_filtered = pts_sub_downsample(valid_sub_indices, :);

fprintf('  基板点云筛选: %d -> %d\n', size(pts_sub_downsample,1), size(pts_sub_filtered,1));

% 多项式曲面拟合
X_sub = pts_sub_filtered(:,1);
Y_sub = pts_sub_filtered(:,2);
Z_sub = pts_sub_filtered(:,3);
M_surface = [ones(size(X_sub)), X_sub, Y_sub, X_sub.^2, Y_sub.^2, X_sub.*Y_sub];
params_substrate = M_surface \ Z_sub;
substrate_surface_params = params_substrate;

s1 = substrate_surface_params(2);
s2 = substrate_surface_params(3);
s3 = substrate_surface_params(4);
s4 = substrate_surface_params(5);
s5 = substrate_surface_params(6);

fprintf('  基板曲面拟合完成\n');

%% 2.4 重新连接焊球分割（与ballCircles对应）
fprintf('  2.4 重新进行焊球连通域分析...\n');
CC = bwconncomp(hanqiuMask_eroded);
stats = regionprops(CC, 'Area', 'PixelIdxList');
areas = [stats.Area];
[~, sortIdx] = sort(areas, 'descend');

%% 2.5 对所有焊球进行高度测量
fprintf('  2.5 对所有焊球进行高度测量...\n');
num_balls_to_measure = length(ballCircles);
vertices_all = zeros(num_balls_to_measure, 3);
heights_all = zeros(num_balls_to_measure, 1);
fallback_flags = false(num_balls_to_measure, 1);

% 建立ballCircles到stats的映射
% 通过圆心位置匹配
ball_to_stats_map = zeros(num_balls_to_measure, 1);

for i = 1:num_balls_to_measure
    xc = ballCircles(i).xc;
    yc = ballCircles(i).yc;
    
    % 找到最接近的连通域
    min_dist = inf;
    best_idx = -1;
    
    for j = 1:length(stats)
        pixels = stats(j).PixelIdxList;
        [rows, cols] = ind2sub(size(hanqiuMask_eroded), pixels);
        centroid = [mean(cols), mean(rows)];
        dist = sqrt((centroid(1) - xc)^2 + (centroid(2) - yc)^2);
        
        if dist < min_dist
            min_dist = dist;
            best_idx = j;
        end
    end
    
    ball_to_stats_map(i) = best_idx;
end

fprintf('  焊球与3D点云匹配完成\n');

% 对每个焊球计算高度
success_count = 0;
fail_count = 0;

for i = 1:num_balls_to_measure
    try
        stats_idx = ball_to_stats_map(i);
        if stats_idx < 1 || stats_idx > length(stats)
            fail_count = fail_count + 1;
            continue;
        end
        
        ball_pixels_i = stats(stats_idx).PixelIdxList;
        pts_i = [X(ball_pixels_i), Y(ball_pixels_i), Z(ball_pixels_i)];
        pts_i = pts_i(~any(isnan(pts_i),2) & pts_i(:,3)~=0, :);
        
        if size(pts_i, 1) < 50
            fail_count = fail_count + 1;
            continue;
        end
        
        % 法向量约束筛选
        k_neighbors_i = 20;
        KDTree_i = KDTreeSearcher(pts_i);
        normals_i = zeros(size(pts_i, 1), 3);
        
        for j = 1:size(pts_i, 1)
            [idx_nbr, ~] = knnsearch(KDTree_i, pts_i(j,:), 'K', min(k_neighbors_i, size(pts_i,1)));
            nbr_points = pts_i(idx_nbr, :);
            coeff = pca(nbr_points);
            normal = coeff(:, 3)';
            if dot(normal, n_ref_normalized) > 0, normal = -normal; end
            normals_i(j, :) = normal;
        end
        
        dot_prods_i = abs(normals_i * n_ref_normalized');
        dot_prods_i = max(min(dot_prods_i, 1), 0);
        thetas_i = acos(dot_prods_i) * 180 / pi;
        theta_thr_i = CONFIG.ball_angle_threshold;
        valid_indices_i = thetas_i <= theta_thr_i;
        pts_i_filtered = pts_i(valid_indices_i, :);
        
        if size(pts_i_filtered, 1) < 50
            fail_count = fail_count + 1;
            continue;
        end
        
        % 迭代曲面拟合
        max_iter_i = CONFIG.surface_fit_iterations;
        pts_i_current = pts_i_filtered;
        
        for iter = 1:max_iter_i
            X_i = pts_i_current(:,1);
            Y_i = pts_i_current(:,2);
            Z_i = pts_i_current(:,3);
            
            M_i = [X_i.^2, Y_i.^2, X_i.*Y_i, X_i, Y_i, ones(size(X_i))];
            p_i = M_i \ Z_i;
            
            Z_pred_i = M_i * p_i;
            residuals_i = abs(Z_i - Z_pred_i);
            
            if iter < max_iter_i
                sigma_r_i = std(residuals_i);
                threshold_i = CONFIG.surface_fit_sigma * sigma_r_i;
                inliers_i = residuals_i < threshold_i;
                
                if sum(~inliers_i) == 0 || sum(inliers_i) < 50
                    break;
                end
                pts_i_current = pts_i_current(inliers_i, :);
            end
        end
        
        % 曲面优化求顶点
        a_i = p_i(1); b_i = p_i(2); c_i = p_i(3);
        d_i = p_i(4); e_i = p_i(5); f_i = p_i(6);
        
        coeff_matrix_i = [2*a_i - 2*s3, c_i - s5;
                         c_i - s5, 2*b_i - 2*s4];
        rhs_i = [s1 - d_i; s2 - e_i];
        
        if abs(det(coeff_matrix_i)) > 1e-10
            vertex_xy_i = coeff_matrix_i \ rhs_i;
            x_vertex_i = vertex_xy_i(1);
            y_vertex_i = vertex_xy_i(2);
            z_vertex_i = a_i*x_vertex_i^2 + b_i*y_vertex_i^2 + c_i*x_vertex_i*y_vertex_i + ...
                        d_i*x_vertex_i + e_i*y_vertex_i + f_i;
            
            z_base_vertex_i = substrate_surface_params(1) + s1*x_vertex_i + s2*y_vertex_i + ...
                             s3*x_vertex_i^2 + s4*y_vertex_i^2 + s5*x_vertex_i*y_vertex_i;
            max_dist_i = z_vertex_i - z_base_vertex_i;
            
            % Hessian矩阵检查
            H11_i = 2*(a_i - s3);
            H22_i = 2*(b_i - s4);
            H12_i = c_i - s5;
            det_H_i = H11_i*H22_i - H12_i^2;
            trace_H_i = H11_i + H22_i;
            is_maximum_i = (det_H_i > 1e-6) && (trace_H_i < -1e-6);
            
            % 掩膜区域检查
            diff_X_i = abs(X - x_vertex_i);
            diff_Y_i = abs(Y - y_vertex_i);
            diff_total_i = diff_X_i + diff_Y_i;
            [~, idx_closest_i] = min(diff_total_i(:));
            in_region_i = ismember(idx_closest_i, ball_pixels_i);
            
            if is_maximum_i && in_region_i
                vertices_all(i, :) = [x_vertex_i, y_vertex_i, z_vertex_i];
                heights_all(i) = max_dist_i;
                fallback_flags(i) = false;
                success_count = success_count + 1;
            else
                % 回退到点云搜索
                num_pts_i = size(pts_i_current, 1);
                dist_to_base_i = zeros(num_pts_i, 1);
                for k = 1:num_pts_i
                    x_k = pts_i_current(k, 1);
                    y_k = pts_i_current(k, 2);
                    z_k = pts_i_current(k, 3);
                    z_base_k = substrate_surface_params(1) + s1*x_k + s2*y_k + ...
                              s3*x_k^2 + s4*y_k^2 + s5*x_k*y_k;
                    dist_to_base_i(k) = z_k - z_base_k;
                end
                [max_dist_i, max_idx_i] = max(dist_to_base_i);
                vertices_all(i, :) = pts_i_current(max_idx_i, :);
                heights_all(i) = max_dist_i;
                fallback_flags(i) = true;
                success_count = success_count + 1;
            end
        else
            % 矩阵奇异，回退
            num_pts_i = size(pts_i_current, 1);
            dist_to_base_i = zeros(num_pts_i, 1);
            for k = 1:num_pts_i
                x_k = pts_i_current(k, 1);
                y_k = pts_i_current(k, 2);
                z_k = pts_i_current(k, 3);
                z_base_k = substrate_surface_params(1) + s1*x_k + s2*y_k + ...
                          s3*x_k^2 + s4*y_k^2 + s5*x_k*y_k;
                dist_to_base_i(k) = z_k - z_base_k;
            end
            [max_dist_i, max_idx_i] = max(dist_to_base_i);
            vertices_all(i, :) = pts_i_current(max_idx_i, :);
            heights_all(i) = max_dist_i;
            fallback_flags(i) = true;
            success_count = success_count + 1;
        end
    catch ME
        fail_count = fail_count + 1;
        fprintf('  警告：焊球#%d测量失败: %s\n', i, ME.message);
        continue;
    end
end

% 清理无效数据
valid_mask = heights_all > 0;

fprintf('\n  【测量统计】\n');
fprintf('    待测量焊球: %d 个\n', num_balls_to_measure);
fprintf('    成功测量: %d 个\n', success_count);
fprintf('    失败: %d 个\n', fail_count);
fprintf('    有效高度数据: %d 个\n', sum(valid_mask));
fprintf('    使用回退方法: %d 个\n', sum(fallback_flags(valid_mask)));

% 检查是否有有效测量
if sum(valid_mask) == 0
    error('未能测量到任何焊球的高度！请检查：\n  1. 3D数据文件(X.mat, Y.mat, Z.mat)是否正确\n  2. 焊球掩膜(hanqiuMask)是否与3D数据对应\n  3. CONFIG参数配置是否合理');
end

fprintf('    平均高度: %.4f mm\n', mean(heights_all(valid_mask)));
fprintf('    标准差: %.4f mm\n', std(heights_all(valid_mask)));
fprintf('    最大高度: %.4f mm\n', max(heights_all(valid_mask)));
fprintf('    最小高度: %.4f mm\n', min(heights_all(valid_mask)));
fprintf('\n');

%% 2.6 生成并保存多焊球顶点分布与高度图
fprintf('  2.6 生成多焊球顶点分布与高度图...\n');

fig2 = figure('Position', [100, 100, 1600, 700], 'Color', 'w');

% 子图1：3D顶点分布（按编号着色）
subplot(1, 2, 1);
valid_indices_plot = find(valid_mask);
vertices_valid = vertices_all(valid_mask, :);
heights_valid = heights_all(valid_mask);
orders_valid = [ballCircles(valid_mask).order];

scatter3(vertices_valid(:,1), vertices_valid(:,2), vertices_valid(:,3), ...
         80, heights_valid, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
hold on;
colorbar;
colormap(jet);
% 设置颜色范围（防止min==max时出错）
h_min = min(heights_valid);
h_max = max(heights_valid);
if abs(h_max - h_min) < 1e-6
    caxis([h_min - 0.001, h_min + 0.001]);
else
    caxis([h_min, h_max]);
end

% 标注编号（每隔几个标注一次，避免拥挤）
step = max(1, floor(length(orders_valid)/20));
for i = 1:step:length(orders_valid)
    text(vertices_valid(i,1)+0.1, vertices_valid(i,2)+0.1, vertices_valid(i,3)+0.1, ...
         sprintf('%d', orders_valid(i)), 'FontSize', 7, 'Color', 'r', ...
         'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.6]);
end

% 绘制基板曲面
x_v_range = linspace(min(vertices_valid(:,1)), max(vertices_valid(:,1)), 30);
y_v_range = linspace(min(vertices_valid(:,2)), max(vertices_valid(:,2)), 30);
[XX_v, YY_v] = meshgrid(x_v_range, y_v_range);
ZZ_v = substrate_surface_params(1) + s1*XX_v + s2*YY_v + ...
       s3*XX_v.^2 + s4*YY_v.^2 + s5*XX_v.*YY_v;
surf(XX_v, YY_v, ZZ_v, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);

xlabel('X/mm', 'FontSize', 11); 
ylabel('Y/mm', 'FontSize', 11); 
zlabel('Z/mm', 'FontSize', 11);
title(sprintf('(a) 多焊球顶点分布 (N=%d)', sum(valid_mask)), ...
    'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend('焊球顶点(按高度着色)', '基板曲面', 'Location', 'best');
grid on; axis equal; axis vis3d; view(3);
set(gca, 'Color', 'w');
hold off;

% 子图2：XY平面高度热力图（标注编号）
subplot(1, 2, 2);
scatter(vertices_valid(:,1), vertices_valid(:,2), 200, heights_valid, ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
hold on;
colorbar;
colormap(gca, jet);
% 设置颜色范围（防止min==max时出错）
h_min = min(heights_valid);
h_max = max(heights_valid);
if abs(h_max - h_min) < 1e-6
    caxis([h_min - 0.001, h_min + 0.001]);
else
    caxis([h_min, h_max]);
end

% 标注所有编号
for i = 1:length(orders_valid)
    text(vertices_valid(i,1), vertices_valid(i,2), sprintf('%d', orders_valid(i)), ...
         'FontSize', 6, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'bottom', 'FontWeight', 'bold', 'Color', 'k');
end

xlabel('X/mm', 'FontSize', 11); 
ylabel('Y/mm', 'FontSize', 11);
title('(b) XY平面高度分布（标注编号）', ...
    'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on;
set(gca, 'Color', 'w');
hold off;

sgtitle(sprintf('多焊球顶点分布与高度差异 (均值=%.4f mm, 标准差=%.4f mm)', ...
        mean(heights_valid), std(heights_valid)), ...
        'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimHei');

saveas(fig2, OUTPUT_FIG2);
fprintf('  ✓ 已保存: %s\n', OUTPUT_FIG2);

fprintf('  【第二部分完成】\n\n');

%% ========== 第三部分：按编号排序并导出Excel ==========
fprintf('\n【第三部分】数据整理与Excel导出...\n');

%% 3.1 按照pic5_05的编号顺序整理数据
fprintf('  3.1 按照编号顺序整理数据...\n');

% 提取编号和高度
ball_orders = [ballCircles.order];
[sorted_orders, sort_idx] = sort(ball_orders);

sorted_heights = heights_all(sort_idx);
sorted_valid = valid_mask(sort_idx);

% 只保留有效测量的焊球
valid_orders = sorted_orders(sorted_valid);
valid_heights = sorted_heights(sorted_valid);

fprintf('  整理完成，有效测量: %d个\n', length(valid_orders));

%% 3.2 构建Excel数据
fprintf('  3.2 构建Excel数据...\n');

% 第一行：标题 + 编号
% 第二行：说明 + 高度值
header_row = cell(1, length(valid_orders) + 1);
data_row = cell(1, length(valid_orders) + 1);

header_row{1} = '焊球编号';
data_row{1} = '相对高度(mm)';

for i = 1:length(valid_orders)
    header_row{i+1} = valid_orders(i);
    data_row{i+1} = valid_heights(i);
end

% 合并为表格
excel_data = [header_row; data_row];

%% 3.3 导出到Excel文件
fprintf('  3.3 导出到Excel文件...\n');

try
    writecell(excel_data, OUTPUT_EXCEL, 'Sheet', 1);
    fprintf('  ✓ 成功导出: %s\n', OUTPUT_EXCEL);
catch
    warning('Excel导出失败，尝试使用CSV格式');
    csv_filename = strrep(OUTPUT_EXCEL, '.xlsx', '.csv');
    writecell(excel_data, csv_filename);
    fprintf('  ✓ 成功导出: %s\n', csv_filename);
end

%% 3.4 显示统计信息
fprintf('\n【统计信息】\n');
fprintf('  总焊球数: %d\n', length(ballCircles));
fprintf('  成功测量: %d\n', length(valid_orders));
fprintf('  平均高度: %.4f mm\n', mean(valid_heights));
fprintf('  标准差: %.4f mm\n', std(valid_heights));
fprintf('  最大高度: %.4f mm (编号#%d)\n', max(valid_heights), valid_orders(valid_heights == max(valid_heights)));
fprintf('  最小高度: %.4f mm (编号#%d)\n', min(valid_heights), valid_orders(valid_heights == min(valid_heights)));

%% 3.5 生成统计可视化图表
fprintf('  3.5 生成统计可视化图表...\n');
fig3 = figure('Position', [100, 100, 1400, 600], 'Color', 'w');

% 子图1：按编号显示高度
subplot(1, 2, 1);
bar(valid_orders, valid_heights, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
hold on;
mean_h = mean(valid_heights);
plot([0, max(valid_orders)+1], [mean_h, mean_h], 'r--', 'LineWidth', 2);
xlabel('焊球编号', 'FontSize', 12, 'FontName', 'SimHei');
ylabel('相对高度 (mm)', 'FontSize', 12, 'FontName', 'SimHei');
title('焊球高度测量结果', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend('测量高度', sprintf('平均值 (%.4f mm)', mean_h), 'Location', 'best');
grid on;
xlim([0, max(valid_orders)+1]);

% 子图2：高度分布直方图
subplot(1, 2, 2);
histogram(valid_heights, 20, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
xlabel('相对高度 (mm)', 'FontSize', 12, 'FontName', 'SimHei');
ylabel('数量', 'FontSize', 12, 'FontName', 'SimHei');
title('高度分布统计', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;

% 添加统计信息
text_str = sprintf('N = %d\n均值 = %.4f mm\n标准差 = %.4f mm', ...
                   length(valid_heights), mean(valid_heights), std(valid_heights));
text(0.65, 0.85, text_str, 'Units', 'normalized', ...
     'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');

sgtitle('焊球高度测量结果总览', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimHei');

saveas(fig3, OUTPUT_FIG3);
fprintf('  ✓ 已保存: %s\n', OUTPUT_FIG3);

fprintf('\n=================================================\n');
fprintf('              处理完成！\n');
fprintf('=================================================\n');
fprintf('\n输出文件：\n');
fprintf('  1. %s (Excel数据)\n', OUTPUT_EXCEL);
fprintf('  2. %s (焊球分割与排序)\n', OUTPUT_FIG1);
fprintf('  3. %s (3D分布与高度)\n', OUTPUT_FIG2);
fprintf('  4. %s (统计图表)\n', OUTPUT_FIG3);
fprintf('\n');

%% 辅助函数：最小二乘圆拟合
function [xc, yc, r] = fit_circle(x, y)
    % 最小二乘法圆拟合
    n = length(x);
    
    A = [2*x, 2*y, ones(n, 1)];
    b = x.^2 + y.^2;
    
    % 求解 [xc; yc; c]
    params = A \ b;
    
    xc = params(1);
    yc = params(2);
    r = sqrt(params(3) + xc^2 + yc^2);
end
