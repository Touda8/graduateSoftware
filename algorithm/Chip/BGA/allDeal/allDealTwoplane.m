function [ballResults, figHandles] = allDealTwoplane(bga_image, X, Y, Z, output_dir, config)
% allDealTwoplane - 焊球检测与测量函数（PCA平面法）
%
% 输入：
%   bga_image - BGA图像矩阵（灰度或RGB）
%   X, Y, Z - 3D坐标矩阵
%   output_dir - 输出图片保存目录
%   config - 可选配置参数结构体
%
% 输出：
%   ballResults - 结构体数组，包含每个焊球的测量结果
%                 .order - 排序编号
%                 .xc, .yc - 圆心坐标(像素)
%                 .r - 半径
%                 .x3d, .y3d, .z3d - 3D顶点坐标(mm)
%                 .height - 相对基板高度(mm)
%                 .row - 所在行号
%                 .success - 是否测量成功
%   figHandles - 生成的图形句柄数组

%% 参数配置
% 设置默认配置
CONFIG = struct();
CONFIG.circularity_threshold = 0.85;
CONFIG.area_min = 300;
CONFIG.area_max = 4000;
CONFIG.edge_distance_threshold = 1;
CONFIG.circle_fit_iterations = 10;
CONFIG.circle_fit_beta = 2.0;
CONFIG.substrate_downsample_ratio = 0.5;
CONFIG.substrate_angle_threshold = 30;
CONFIG.ball_angle_threshold = 45;
CONFIG.surface_fit_iterations = 100;
CONFIG.surface_fit_sigma = 2.5;
CONFIG.z_flip = 160;

% 如果提供了自定义配置，覆盖默认值
if nargin >= 6 && ~isempty(config)
    fields = fieldnames(config);
    for i = 1:length(fields)
        CONFIG.(fields{i}) = config.(fields{i});
    end
end

% Z轴翻转处理
if isfield(CONFIG, 'z_flip') && CONFIG.z_flip > 0
    Z = CONFIG.z_flip - Z;
end

fprintf('\n========== 焊球检测与测量开始 ==========\n');

%% ========== 第一部分：焊球检测与编号 ==========
fprintf('【第一部分】焊球检测、质心提取与编号...\n');

%% 1.1 读取2D图像
I_original = bga_image;
if size(I_original, 3) == 3
    I_original = rgb2gray(I_original);
end
%% 1.2 芯片区域粗定位
level_chip = graythresh(I_original);
BW_chip = imbinarize(I_original, level_chip);
se_chip = strel('square', 50);
BW_eroded = imerode(BW_chip, se_chip);
BW_morph = imdilate(BW_eroded, se_chip);
BW_inverted = ~BW_morph;
CC_chip = bwconncomp(BW_inverted);
stats_chip = regionprops(CC_chip, 'Area');
[~, maxIdx_chip] = max([stats_chip.Area]);
BW_max_chip = false(size(BW_inverted));
BW_max_chip(CC_chip.PixelIdxList{maxIdx_chip}) = true;
mask_chip = ones(size(I_original)) * (-1);
mask_chip(BW_max_chip) = 1;
I_gray = I_original;
I_gray(mask_chip == -1) = 0;

%% 1.3 基板区域分割
roi_pixels = I_gray(mask_chip == 1);
level = graythresh(roi_pixels);
BW = imbinarize(I_gray, level);
BW_substrate = ~BW;
CC = bwconncomp(BW_substrate);
stats = regionprops(CC, 'Area');
[~, maxIdx] = max([stats.Area]);
BW_substrate_main = false(size(BW));
BW_substrate_main(CC.PixelIdxList{maxIdx}) = true;

%% 1.4 形态学闭运算
[img_h, img_w] = size(I_gray);
rho = min(img_h, img_w) / 1000;
d = 2;
r = ceil(rho * d);
r = max(r, 3);
se = strel('disk', r);
BW_substrate_closed = imclose(BW_substrate_main, se);

%% 1.5 焊球候选区域检测
BW_high = double(abs(BW_substrate_closed-1).*255);
BW_balls_all = BW_high;
CC_balls = bwconncomp(BW_balls_all);
stats_balls = regionprops(CC_balls, 'Area', 'Perimeter', 'Centroid', 'BoundingBox');

%% 1.6 圆形度与面积约束筛选
circularity = zeros(length(stats_balls), 1);
for i = 1:length(stats_balls)
    A = stats_balls(i).Area;
    P = stats_balls(i).Perimeter;
    if P > 0
        circularity(i) = 4 * pi * A / (P^2);
    end
end

valid_geometry = (circularity > CONFIG.circularity_threshold) & ...
                 ([stats_balls.Area]' > CONFIG.area_min) & ...
                 ([stats_balls.Area]' < CONFIG.area_max);

%% 1.7 边界距离约束
BW_substrate_boundary = bwperim(BW_substrate_closed);
D = bwdist(BW_substrate_boundary);
valid_distance = false(length(stats_balls), 1);

for i = 1:length(stats_balls)
    x = round(stats_balls(i).Centroid(1));
    y = round(stats_balls(i).Centroid(2));
    x = max(1, min(x, size(D, 2)));
    y = max(1, min(y, size(D, 1)));
    if D(y, x) >= CONFIG.edge_distance_threshold
        valid_distance(i) = true;
    end
end

%% 1.8 生成最终焊球掩膜
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

%% 1.9 基于连通域质心构建焊球圆心（不做圆拟合）
num_balls = length(valid_indices);
ballCircles = struct('xc', {}, 'yc', {}, 'r', {}, 'ballIdx', {});

for idx = 1:num_balls
    ball_idx_current = valid_indices(idx);

    % 使用连通域质心作为圆心
    centroid_current = stats_balls(ball_idx_current).Centroid;
    xc_current = centroid_current(1);
    yc_current = centroid_current(2);

    % 用等效半径替代拟合半径（保持后续可视化兼容）
    area_current = stats_balls(ball_idx_current).Area;
    r_equiv = sqrt(area_current / pi);

    ballCircles(idx).xc = xc_current;
    ballCircles(idx).yc = yc_current;
    ballCircles(idx).r = r_equiv;
    ballCircles(idx).ballIdx = ball_idx_current;
end

fprintf('  完成 %d 个焊球的质心提取（圆心=连通域质心）\n', length(ballCircles));

%% 1.10 芯片旋转矫正与焊球排序
BW_chip_mask = (mask_chip == 1);
BW_edge = edge(BW_chip_mask, 'Canny');
[edge_y, edge_x] = find(BW_edge);

if length(edge_x) >= 100
    edge_points = [edge_x, edge_y];
    edge_points_centered = edge_points - mean(edge_points, 1);
    [coeff, ~, ~] = pca(edge_points_centered);
    principal_direction = coeff(:, 1);
    main_direction_angle = atan2d(principal_direction(2), principal_direction(1));
    rotation_angle = -main_direction_angle;
else
    rotation_angle = 0;
end

fprintf('  旋转角度: %.2f 度\n', rotation_angle);

% 旋转变换
theta = deg2rad(rotation_angle);
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
img_center = [size(I_original, 2)/2, size(I_original, 1)/2];

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

ball_order = zeros(num_balls_total, 1);
global_index = 1;

for row = 1:num_rows
    row_indices = find(row_assignments == row);
    row_x_coords = X_coords(row_indices);
    [~, sort_order] = sort(row_x_coords);
    sorted_indices = row_indices(sort_order);
    
    for j = 1:length(sorted_indices)
        ball_order(sorted_indices(j)) = global_index;
        global_index = global_index + 1;
    end
end

for idx = 1:num_balls_total
    ballCircles(idx).order = ball_order(idx);
    ballCircles(idx).row = row_assignments(idx);
end

fprintf('  焊球排序完成\n');

%% ========== 第二部分：3D测量 ==========
fprintf('【第二部分】3D曲面拟合与高度测量...\n');

%% 2.1 准备掩膜数据
hanqiuMask = logical(BW_balls);
mask_chip_save = (mask_chip+1)/2;
I_masked = logical(mask_chip_save);


se_erode = strel('disk', 2);
I_masked = imerode(I_masked, se_erode);
hanqiuMask_eroded = imerode(hanqiuMask, se_erode);

se_dilate = strel('disk', 5);
hanqiuMask_dilated = imdilate(hanqiuMask, se_dilate);

%% 2.2 基板平面拟合（PCA分析法）
fprintf('  基板平面拟合（PCA分析法）...\n');
validDataMask = (Z ~= 0) & ~isnan(Z);
substrateMask = I_masked & ~hanqiuMask_dilated & validDataMask;
idx_sub = find(substrateMask);

pts_sub_init = [X(idx_sub), Y(idx_sub), Z(idx_sub)];
pts_sub_init = pts_sub_init(~any(isnan(pts_sub_init),2), :);

if size(pts_sub_init, 1) < 30
    error('基板有效点云过少，无法进行PCA平面拟合');
end

% PCA平面：最小主成分方向作为法向量
centroid_sub = mean(pts_sub_init, 1);
[coeff_sub, ~, ~] = pca(pts_sub_init);
n_sub = coeff_sub(:, 3);
n_sub = n_sub / norm(n_sub);

% 统一方向：优先令z分量为负
if n_sub(3) > 0
    n_sub = -n_sub;
end

% 平面方程: n_sub' * [x y z]' + d_sub = 0
d_sub = -dot(n_sub, centroid_sub');

fprintf('  基板平面法向量: [%.6f, %.6f, %.6f]\n', n_sub(1), n_sub(2), n_sub(3));
fprintf('  基板平面质心: [%.6f, %.6f, %.6f]\n', centroid_sub(1), centroid_sub(2), centroid_sub(3));
fprintf('  基板平面拟合完成\n');

%% 2.3 焊球连通域分析
CC = bwconncomp(hanqiuMask_eroded);
stats = regionprops(CC, 'Area', 'PixelIdxList');

%% 2.4 对所有焊球进行高度测量（单焊球PCA平面到基板PCA平面）
fprintf('  对所有焊球进行高度测量（单焊球PCA平面测距）...\n');
num_balls_to_measure = length(ballCircles);
vertices_all = zeros(num_balls_to_measure, 3);
heights_all = zeros(num_balls_to_measure, 1);
ball_normals_all = nan(num_balls_to_measure, 3);
centroids_2d_all = nan(num_balls_to_measure, 2);
centroids_3d_all = nan(num_balls_to_measure, 3);

% 建立映射
ball_to_stats_map = zeros(num_balls_to_measure, 1);

for i = 1:num_balls_to_measure
    center_2d = [ballCircles(i).xc, ballCircles(i).yc];
    min_dist = inf;
    best_idx = -1;
    
    for j = 1:length(stats)
        [rows, cols] = ind2sub(size(hanqiuMask_eroded), stats(j).PixelIdxList);
        centroid_j = [mean(cols), mean(rows)];
        dist = norm(center_2d - centroid_j);
        
        if dist < min_dist
            min_dist = dist;
            best_idx = j;
        end
    end
    
    ball_to_stats_map(i) = best_idx;
end

% 测量高度（简化方法）
success_count = 0;
fail_count = 0;

for i = 1:num_balls_to_measure
    try
        stats_idx = ball_to_stats_map(i);
        if stats_idx < 1 || stats_idx > length(stats)
            fail_count = fail_count + 1;
            continue;
        end
        
        % 获取焊球点云
        ball_pixels_i = stats(stats_idx).PixelIdxList;
        pts_ball = [X(ball_pixels_i), Y(ball_pixels_i), Z(ball_pixels_i)];
        pts_ball = pts_ball(~any(isnan(pts_ball),2) & pts_ball(:,3)~=0, :);
        
        if size(pts_ball, 1) < 20  % 至少需要20个点
            fail_count = fail_count + 1;
            continue;
        end
        
        % 单焊球PCA平面
        centroid_ball = mean(pts_ball, 1);
        [coeff_ball, ~, ~] = pca(pts_ball);
        n_ball = coeff_ball(:, 3);
        n_ball = n_ball / norm(n_ball);
        if dot(n_ball, n_sub) < 0
            n_ball = -n_ball;
        end

        % hanqiuMask_eroded 的白色连通域质心（2D）
        [rows_i, cols_i] = ind2sub(size(hanqiuMask_eroded), ball_pixels_i);
        centroid_2d = [mean(cols_i), mean(rows_i)];

        % 焊球质心到基板平面的距离（单位与XYZ一致，通常mm）
        signed_dist = dot(n_sub, centroid_ball') + d_sub;
        height = abs(signed_dist);

        % 保存结果
        vertices_all(i, :) = centroid_ball;
        heights_all(i) = height;
        ball_normals_all(i, :) = n_ball';
        centroids_2d_all(i, :) = centroid_2d;
        centroids_3d_all(i, :) = centroid_ball;
        success_count = success_count + 1;
        
    catch ME
        fail_count = fail_count + 1;
        fprintf('  警告：焊球#%d测量失败: %s\n', i, ME.message);
        continue;
    end
end

% 清理无效数据
valid_mask = heights_all > 0;

fprintf('  测量完成: 成功%d个, 失败%d个\n', success_count, fail_count);

%% 组织输出结果
ballResults = struct('order', {}, 'xc', {}, 'yc', {}, 'r', {}, ...
                     'x3d', {}, 'y3d', {}, 'z3d', {}, 'height', {}, ...
                     'row', {}, 'success', {});

for i = 1:num_balls_to_measure
    ballResults(i).order = ballCircles(i).order;
    ballResults(i).xc = ballCircles(i).xc;
    ballResults(i).yc = ballCircles(i).yc;
    ballResults(i).r = ballCircles(i).r;
    ballResults(i).row = ballCircles(i).row;
    ballResults(i).success = valid_mask(i);
    
    if valid_mask(i)
        ballResults(i).x3d = vertices_all(i, 1);
        ballResults(i).y3d = vertices_all(i, 2);
        ballResults(i).z3d = vertices_all(i, 3);
        ballResults(i).height = heights_all(i);
    else
        ballResults(i).x3d = NaN;
        ballResults(i).y3d = NaN;
        ballResults(i).z3d = NaN;
        ballResults(i).height = NaN;
    end
end

%% 生成并保存图形
if nargin >= 5 && ~isempty(output_dir)
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    fprintf('  生成结果图...\n');
    figHandles = [];
    
    % 图1：焊球分割与排序
    fig1 = figure('Position', [100, 100, 1600, 800], 'Color', 'w', 'Visible', 'off');
    
    subplot(1, 2, 1);
    imshow(I_original);
    hold on;
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
    title(sprintf('焊球分割 (共%d个)', length(ballCircles)), 'FontSize', 13, 'FontWeight', 'bold');
    hold off;
    
    subplot(1, 2, 2);
    imshow(I_original);
    hold on;
    cmap = lines(num_rows);
    for idx = 1:length(ballCircles)
        xc = ballCircles(idx).xc;
        yc = ballCircles(idx).yc;
        row_num = ballCircles(idx).row;
        order_num = ballCircles(idx).order;
        plot(xc, yc, 'o', 'MarkerSize', 10, 'Color', cmap(row_num,:), 'LineWidth', 2);
        text(xc, yc, num2str(order_num), 'Color', 'white', 'FontSize', 8, ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    title(sprintf('焊球排序 (共%d行)', num_rows), 'FontSize', 13, 'FontWeight', 'bold');
    hold off;
    
    saveas(fig1, fullfile(output_dir, '焊球分割与排序.png'));
    figHandles = [figHandles, fig1];
    
    % 图2：3D分布
    if sum(valid_mask) > 0
        fig2 = figure('Position', [100, 100, 1200, 700], 'Color', 'w', 'Visible', 'off');
        
        vertices_valid = vertices_all(valid_mask, :);
        heights_valid = heights_all(valid_mask);
        
        scatter3(vertices_valid(:,1), vertices_valid(:,2), vertices_valid(:,3), ...
                 80, heights_valid, 'filled', 'MarkerEdgeColor', 'k');
        hold on;
        colorbar;
        colormap(jet);
        
        h_min = min(heights_valid);
        h_max = max(heights_valid);
        if abs(h_max - h_min) < 1e-6
            caxis([h_min - 0.001, h_min + 0.001]);
        else
            caxis([h_min, h_max]);
        end
        
        % 绘制基板（PCA平面）
        x_v_range = linspace(min(vertices_valid(:,1)), max(vertices_valid(:,1)), 30);
        y_v_range = linspace(min(vertices_valid(:,2)), max(vertices_valid(:,2)), 30);
        [XX_v, YY_v] = meshgrid(x_v_range, y_v_range);
        if abs(n_sub(3)) > 1e-10
            ZZ_v = -(n_sub(1)*XX_v + n_sub(2)*YY_v + d_sub) / n_sub(3);
            surf(XX_v, YY_v, ZZ_v, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);
        end
        
        xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
        title(sprintf('焊球3D分布 (N=%d)', sum(valid_mask)), 'FontSize', 13, 'FontWeight', 'bold');
        grid on; axis equal; view(3);
        hold off;
        
        saveas(fig2, fullfile(output_dir, '焊球3D分布.png'));
        figHandles = [figHandles, fig2];

        % 图3：hanqiuMask_eroded 质心标注图
        fig3 = figure('Position', [100, 100, 1400, 900], 'Color', 'w', 'Visible', 'off');
        imshow(hanqiuMask_eroded);
        hold on;
        title('hanqiuMask\_eroded 质心标注', 'FontSize', 13, 'FontWeight', 'bold');
        valid_ids = find(valid_mask);
        for ii = 1:length(valid_ids)
            k = valid_ids(ii);
            cx = centroids_2d_all(k, 1);
            cy = centroids_2d_all(k, 2);
            if ~isnan(cx) && ~isnan(cy)
                plot(cx, cy, 'r+', 'MarkerSize', 10, 'LineWidth', 1.8);
                text(cx+3, cy-3, sprintf('#%d', ballResults(k).order), ...
                    'Color', 'y', 'FontSize', 8, 'FontWeight', 'bold', ...
                    'BackgroundColor', [0 0 0 0.5]);
            end
        end
        hold off;
        saveas(fig3, fullfile(output_dir, 'hanqiuMask_eroded_质心标注.png'));
        figHandles = [figHandles, fig3];

        % 图4：法向夹角统计图
        fig4 = figure('Position', [100, 100, 1200, 500], 'Color', 'w', 'Visible', 'off');
        valid_normals = ball_normals_all(valid_mask, :);
        cos_t = valid_normals * n_sub;
        cos_t = max(min(cos_t, 1), -1);
        ang_deg = acosd(cos_t);

        subplot(1, 2, 1);
        histogram(ang_deg, 20);
        xlabel('夹角 / 度'); ylabel('数量');
        title('焊球平面法向与基板法向夹角分布');
        grid on;

        subplot(1, 2, 2);
        scatter(1:length(ang_deg), ang_deg, 40, heights_valid, 'filled');
        colorbar;
        xlabel('有效焊球索引'); ylabel('法向夹角 / 度');
        title('夹角-高度关联图');
        grid on;

        saveas(fig4, fullfile(output_dir, '法向夹角统计.png'));
        figHandles = [figHandles, fig4];

        % 图5：每个焊球质心局部截图
        centroid_dir = fullfile(output_dir, 'centroid_crops');
        if ~exist(centroid_dir, 'dir')
            mkdir(centroid_dir);
        end

        crop_half = 30;
        [h_img, w_img] = size(I_original);
        for ii = 1:length(valid_ids)
            k = valid_ids(ii);
            cx = round(centroids_2d_all(k, 1));
            cy = round(centroids_2d_all(k, 2));
            if isnan(cx) || isnan(cy)
                continue;
            end

            x1 = max(1, cx - crop_half);
            x2 = min(w_img, cx + crop_half);
            y1 = max(1, cy - crop_half);
            y2 = min(h_img, cy + crop_half);
            crop_img = I_original(y1:y2, x1:x2);

            fig_tmp = figure('Visible', 'off', 'Color', 'w');
            imshow(crop_img);
            hold on;
            plot(cx - x1 + 1, cy - y1 + 1, 'r+', 'MarkerSize', 10, 'LineWidth', 1.8);
            text(5, 12, sprintf('order=%d, h=%.4fmm', ballResults(k).order, heights_all(k)), ...
                'Color', 'y', 'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', [0 0 0 0.5]);
            hold off;

            saveas(fig_tmp, fullfile(centroid_dir, sprintf('ball_%03d_order_%03d.png', k, ballResults(k).order)));
            close(fig_tmp);
        end
    end
    
    fprintf('  ✓ 图片已保存至: %s\n', output_dir);
else
    figHandles = [];
end

fprintf('========== 焊球检测与测量完成 ==========\n\n');

end
