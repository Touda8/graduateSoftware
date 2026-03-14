% fig5_10_2.m - 图5-10-2: 基于迭代剔除的封装本体边界亚像素级精化
% (a) 粗定位矩形边界
% (b) Canny边缘点分布
% (c) 迭代剔除异常边缘点
% (d) 精化边界与实际轮廓对比
%
% 输入：I_gray - 原始灰度图像
%       mask_chip - 芯片区域掩膜
% 输出：body_boundary - 封装本体精确边界参数
%       body_mask - 封装本体掩膜
%
% 使用方法：运行测试脚本 test_fig5_10_2

function [body_boundary, body_mask] = QFP_Seg_Step2_BodyBoundary(I_gray, mask_chip)
% 输入：I_gray - 原始灰度图像
%       mask_chip - 芯片区域掩膜
% 输出：body_boundary - 封装本体精确边界参数 [top, bottom, left, right]
%       body_mask - 封装本体掩膜

% 确保 I_gray 是 double 类型
if ~isa(I_gray, 'double')
    I_gray = im2double(I_gray);
end

%% 1. 在芯片掩膜内再次进行阈值分割（分离封装本体）
I_chip_region = I_gray .* double(mask_chip);
I_chip_region(I_chip_region == 0) = NaN;

% Otsu阈值分割（封装本体为暗色）
threshold = graythresh(I_chip_region(~isnan(I_chip_region)));
body_binary = I_chip_region < threshold;
body_binary(isnan(I_chip_region)) = false;

% 形态学闭运算填充空洞
se = strel('disk', 5);
body_binary = imclose(body_binary, se);

% 提取最大连通域
CC = bwconncomp(body_binary);
numPixels = cellfun(@numel, CC.PixelIdxList);
[~, idx_max] = max(numPixels);
body_mask = false(size(I_gray));
body_mask(CC.PixelIdxList{idx_max}) = true;

%% 2. 获取粗边界（最小外接矩形）
props = regionprops(body_mask, 'BoundingBox');
bbox = props.BoundingBox; % [x, y, width, height]
x_min_coarse = bbox(1);
y_min_coarse = bbox(2);
x_max_coarse = bbox(1) + bbox(3);
y_max_coarse = bbox(2) + bbox(4);

%% 3. Canny边缘检测
edges = edge(I_gray, 'Canny');

%% 4. 对四条边界分别进行迭代最小二乘直线拟合
search_width = 20; % 搜索带宽度
alpha = 2.5; % 离群点阈值系数
max_iter = 5; % 最大迭代次数

% 初始化边界参数
boundary_params = struct();

% (1) 上边界：y = k*x + b
fprintf('拟合上边界...\n');
y_range = [max(1, y_min_coarse - search_width), y_min_coarse + search_width];
x_range = [x_min_coarse, x_max_coarse];
[row, col] = find(edges);
edge_points_top = [col(row >= y_range(1) & row <= y_range(2) & ...
                         col >= x_range(1) & col <= x_range(2)), ...
                   row(row >= y_range(1) & row <= y_range(2) & ...
                         col >= x_range(1) & col <= x_range(2))];
[k_top, b_top, valid_top] = iterative_line_fit(edge_points_top, alpha, max_iter);
boundary_params.top.k = k_top;
boundary_params.top.b = b_top;
boundary_params.top.points = edge_points_top(valid_top, :);

% (2) 下边界：y = k*x + b
fprintf('拟合下边界...\n');
y_range = [y_max_coarse - search_width, min(size(I_gray,1), y_max_coarse + search_width)];
edge_points_bottom = [col(row >= y_range(1) & row <= y_range(2) & ...
                            col >= x_range(1) & col <= x_range(2)), ...
                      row(row >= y_range(1) & row <= y_range(2) & ...
                            col >= x_range(1) & col <= x_range(2))];
[k_bottom, b_bottom, valid_bottom] = iterative_line_fit(edge_points_bottom, alpha, max_iter);
boundary_params.bottom.k = k_bottom;
boundary_params.bottom.b = b_bottom;
boundary_params.bottom.points = edge_points_bottom(valid_bottom, :);

% (3) 左边界：x = k*y + b
fprintf('拟合左边界...\n');
x_range_left = [max(1, x_min_coarse - search_width), x_min_coarse + search_width];
y_range_left = [y_min_coarse, y_max_coarse];
edge_points_left = [row(col >= x_range_left(1) & col <= x_range_left(2) & ...
                          row >= y_range_left(1) & row <= y_range_left(2)), ...
                    col(col >= x_range_left(1) & col <= x_range_left(2) & ...
                          row >= y_range_left(1) & row <= y_range_left(2))];
[k_left, b_left, valid_left] = iterative_line_fit(edge_points_left, alpha, max_iter);
boundary_params.left.k = k_left;
boundary_params.left.b = b_left;
boundary_params.left.points = edge_points_left(valid_left, :);

% (4) 右边界：x = k*y + b
fprintf('拟合右边界...\n');
x_range_right = [x_max_coarse - search_width, min(size(I_gray,2), x_max_coarse + search_width)];
edge_points_right = [row(col >= x_range_right(1) & col <= x_range_right(2) & ...
                           row >= y_range_left(1) & row <= y_range_left(2)), ...
                     col(col >= x_range_right(1) & col <= x_range_right(2) & ...
                           row >= y_range_left(1) & row <= y_range_left(2))];
[k_right, b_right, valid_right] = iterative_line_fit(edge_points_right, alpha, max_iter);
boundary_params.right.k = k_right;
boundary_params.right.b = b_right;
boundary_params.right.points = edge_points_right(valid_right, :);

%% 5. 构造输出边界结构
body_boundary.top = boundary_params.top;
body_boundary.bottom = boundary_params.bottom;
body_boundary.left = boundary_params.left;
body_boundary.right = boundary_params.right;

% 计算边界交点作为精确矩形角点
x_left = mean(boundary_params.left.points(:, 2));
x_right = mean(boundary_params.right.points(:, 2));
y_top = k_top * mean([x_min_coarse, x_max_coarse]) + b_top;
y_bottom = k_bottom * mean([x_min_coarse, x_max_coarse]) + b_bottom;

body_boundary.rect = [x_left, y_top, x_right - x_left, y_bottom - y_top];

%% 6. 可视化
figure('Position', [100, 100, 1400, 800], 'Color', 'w');

% (a) 粗定位矩形边界
subplot(2, 2, 1);
imshow(I_gray); hold on;
rectangle('Position', bbox, 'EdgeColor', 'r', 'LineWidth', 2, 'LineStyle', '--');
title('(a) 粗定位矩形边界', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);
legend('粗定位边界', 'Location', 'best');

% (b) Canny边缘点分布
subplot(2, 2, 2);
imshow(I_gray); hold on;
plot(edge_points_top(:,1), edge_points_top(:,2), 'g.', 'MarkerSize', 3);
plot(edge_points_bottom(:,1), edge_points_bottom(:,2), 'g.', 'MarkerSize', 3);
plot(edge_points_left(:,2), edge_points_left(:,1), 'g.', 'MarkerSize', 3);
plot(edge_points_right(:,2), edge_points_right(:,1), 'g.', 'MarkerSize', 3);
title('(b) Canny边缘点分布', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);

% (c) 迭代剔除异常边缘点
subplot(2, 2, 3);
imshow(I_gray); hold on;
% 显示被剔除的点
outlier_top = edge_points_top(~valid_top, :);
outlier_bottom = edge_points_bottom(~valid_bottom, :);
outlier_left = edge_points_left(~valid_left, :);
outlier_right = edge_points_right(~valid_right, :);
if ~isempty(outlier_top), plot(outlier_top(:,1), outlier_top(:,2), 'r.', 'MarkerSize', 5); end
if ~isempty(outlier_bottom), plot(outlier_bottom(:,1), outlier_bottom(:,2), 'r.', 'MarkerSize', 5); end
if ~isempty(outlier_left), plot(outlier_left(:,2), outlier_left(:,1), 'r.', 'MarkerSize', 5); end
if ~isempty(outlier_right), plot(outlier_right(:,2), outlier_right(:,1), 'r.', 'MarkerSize', 5); end
% 显示保留的点
plot(boundary_params.top.points(:,1), boundary_params.top.points(:,2), 'g.', 'MarkerSize', 3);
plot(boundary_params.bottom.points(:,1), boundary_params.bottom.points(:,2), 'g.', 'MarkerSize', 3);
plot(boundary_params.left.points(:,2), boundary_params.left.points(:,1), 'g.', 'MarkerSize', 3);
plot(boundary_params.right.points(:,2), boundary_params.right.points(:,1), 'g.', 'MarkerSize', 3);
title('(c) 迭代剔除异常边缘点', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);
legend('异常点', '有效点', 'Location', 'best');

% (d) 精化边界与实际轮廓对比
subplot(2, 2, 4);
imshow(I_gray); hold on;
% 绘制精化后的直线
x_plot = linspace(x_min_coarse, x_max_coarse, 100);
y_top_plot = k_top * x_plot + b_top;
y_bottom_plot = k_bottom * x_plot + b_bottom;
plot(x_plot, y_top_plot, 'b-', 'LineWidth', 2);
plot(x_plot, y_bottom_plot, 'b-', 'LineWidth', 2);

y_plot = linspace(y_min_coarse, y_max_coarse, 100);
x_left_plot = k_left * y_plot + b_left;
x_right_plot = k_right * y_plot + b_right;
plot(x_left_plot, y_plot, 'b-', 'LineWidth', 2);
plot(x_right_plot, y_plot, 'b-', 'LineWidth', 2);

% 绘制粗边界对比
rectangle('Position', bbox, 'EdgeColor', 'r', 'LineWidth', 1, 'LineStyle', '--');
title('(d) 精化边界与实际轮廓对比', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);
legend('精化边界', '粗定位边界', 'Location', 'best');

sgtitle('图5-10-2 基于迭代剔除的封装本体边界亚像素级精化', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

fprintf('边界精化完成\n');

end

%% 辅助函数：迭代最小二乘直线拟合
function [k, b, valid_idx] = iterative_line_fit(points, alpha, max_iter)
% points: [x, y] 坐标
% alpha: 离群点阈值系数
% max_iter: 最大迭代次数

if isempty(points) || size(points, 1) < 2
    k = 0; b = 0; valid_idx = false(size(points, 1), 1);
    return;
end

valid_idx = true(size(points, 1), 1);
x = points(:, 1);
y = points(:, 2);

for iter = 1:max_iter
    % 最小二乘拟合
    x_valid = x(valid_idx);
    y_valid = y(valid_idx);
    
    if length(x_valid) < 2
        break;
    end
    
    % y = kx + b
    M = [x_valid, ones(length(x_valid), 1)];
    p = M \ y_valid;
    k = p(1);
    b = p(2);
    
    % 计算残差
    distances = abs(k * x - y + b) ./ sqrt(k^2 + 1);
    
    % 计算标准差
    sigma_d = std(distances(valid_idx));
    
    % 剔除离群点
    threshold = alpha * sigma_d;
    valid_idx_new = distances <= threshold;
    
    % 检查收敛
    if sum(valid_idx_new ~= valid_idx) == 0
        break;
    end
    
    valid_idx = valid_idx_new;
end

fprintf('  迭代%d次，保留 %d/%d 点\n', iter, sum(valid_idx), length(valid_idx));

end
