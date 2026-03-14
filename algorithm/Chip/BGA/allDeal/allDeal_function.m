function [ballResults, figHandles] = allDeal_function(bga_image, X, Y, Z, output_dir, config)
% allDeal_function - 焊球检测、测量与数据导出函数版本
%
% 输入：
%   bga_image_path - BGA图像路径
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
CONFIG.substrate_downsample_ratio = 0.1;
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
fprintf('【第一部分】焊球检测、圆拟合与编号...\n');

%% 1.1 读取2D图像
fprintf(' 读取图像\n');
I_original = bga_image;


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

%% 1.9 圆拟合
num_balls = length(valid_indices);
ballCircles = struct('xc', {}, 'yc', {}, 'r', {}, 'ballIdx', {});
edges = edge(double(BW_balls), 'Canny');

for idx = 1:num_balls
    ball_idx_current = valid_indices(idx);
    mask_current = false(size(BW_balls));
    mask_current(CC_balls.PixelIdxList{ball_idx_current}) = true;
    mask_dilated = imdilate(mask_current, strel('disk', 5));
    edge_mask_current = mask_dilated & edges;
    [y_edge, x_edge] = find(edge_mask_current);
    
    if length(x_edge) < 10
        continue;
    end
    
    points = [x_edge, y_edge];
    xc_init = mean(x_edge);
    yc_init = mean(y_edge);
    r_init = mean(sqrt((x_edge - xc_init).^2 + (y_edge - yc_init).^2));
    
    for iter = 1:CONFIG.circle_fit_iterations
        residuals = sqrt((points(:,1) - xc_init).^2 + (points(:,2) - yc_init).^2) - r_init;
        sigma = std(residuals);
        threshold = CONFIG.circle_fit_beta * sigma;
        inliers = abs(residuals) < threshold;
        
        if sum(inliers) < 10
            break;
        end
        
        points_inliers = points(inliers, :);
        xc_init = mean(points_inliers(:,1));
        yc_init = mean(points_inliers(:,2));
        r_init = mean(sqrt((points_inliers(:,1) - xc_init).^2 + (points_inliers(:,2) - yc_init).^2));
    end
    
    ballCircles(idx).xc = xc_init;
    ballCircles(idx).yc = yc_init;
    ballCircles(idx).r = r_init;
    ballCircles(idx).ballIdx = ball_idx_current;
end

fprintf('  完成 %d 个焊球的圆拟合\n', length(ballCircles));

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
hanqiuMask_eroded = imerode(hanqiuMask, se_erode);

se_dilate = strel('disk', 5);
hanqiuMask_dilated = imdilate(hanqiuMask, se_dilate);

%% 2.2 基板曲面拟合
fprintf('  基板曲面拟合...\n');
validDataMask = (Z ~= 0) & ~isnan(Z);
substrateMask = I_masked & ~hanqiuMask_dilated & validDataMask;
idx_sub = find(substrateMask);

pts_sub_init = [X(idx_sub), Y(idx_sub), Z(idx_sub)];
pts_sub_init = pts_sub_init(~any(isnan(pts_sub_init),2), :);

M_plane = [pts_sub_init(:,1), pts_sub_init(:,2), ones(size(pts_sub_init,1),1)];
B_plane = M_plane \ pts_sub_init(:,3);
A_ref = -B_plane(1);
B_ref = -B_plane(2);
n_ref = [A_ref, B_ref, -1];
n_ref_normalized = n_ref / norm(n_ref);
if n_ref_normalized(3) > 0, n_ref_normalized = -n_ref_normalized; end

% 基板点云降采样
downsample_ratio = CONFIG.substrate_downsample_ratio;
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
theta_thr_sub = CONFIG.substrate_angle_threshold;
valid_sub_indices = thetas_sub <= theta_thr_sub;
pts_sub_filtered = pts_sub_downsample(valid_sub_indices, :);

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

%% 2.3 焊球连通域分析
CC = bwconncomp(hanqiuMask_eroded);
stats = regionprops(CC, 'Area', 'PixelIdxList');
areas = [stats.Area];
[~, sortIdx] = sort(areas, 'descend');
fprintf('  对所有焊球进行高度测量...\n');
num_balls_to_measure = length(ballCircles);
vertices_all = zeros(num_balls_to_measure, 3);
heights_all = zeros(num_balls_to_measure, 1);
fallback_flags = false(num_balls_to_measure, 1);

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

% 测量高度
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
    catch
        fail_count = fail_count + 1;
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
        
        % 绘制基板
        x_v_range = linspace(min(vertices_valid(:,1)), max(vertices_valid(:,1)), 30);
        y_v_range = linspace(min(vertices_valid(:,2)), max(vertices_valid(:,2)), 30);
        [XX_v, YY_v] = meshgrid(x_v_range, y_v_range);
        ZZ_v = substrate_surface_params(1) + s1*XX_v + s2*YY_v + ...
               s3*XX_v.^2 + s4*YY_v.^2 + s5*XX_v.*YY_v;
        surf(XX_v, YY_v, ZZ_v, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);
        
        xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
        title(sprintf('焊球3D分布 (N=%d)', sum(valid_mask)), 'FontSize', 13, 'FontWeight', 'bold');
        grid on; axis equal; view(3);
        hold off;
        
        saveas(fig2, fullfile(output_dir, '焊球3D分布.png'));
        figHandles = [figHandles, fig2];
    end
    
    fprintf('  ✓ 图片已保存至: %s\n', output_dir);
else
    figHandles = [];
end

fprintf('========== 焊球检测与测量完成 ==========\n\n');

end
