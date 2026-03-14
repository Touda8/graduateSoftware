% pic5_08.m - 图5-8: 基于曲面优化的焊球顶点定位算法
% 
% 核心思想：在拟合的焊球曲面上求解相对基板曲面距离最大的点
% 
% 优化目标：max_{(X,Y)} [Z_ball(X,Y) - Z_substrate(X,Y)]
% 求解方法：偏导数=0 → 线性方程组 → 解析解
% 
% 算法优势：
%   - 利用曲面拟合结果，理论上更精确
%   - 不受点云采样密度限制
%   - 处理基板翘曲和焊球变形
% 
% (a) 焊球曲面与基板平面空间关系
% (b) 焊球曲面拟合与顶点极值点分析
% (c) 顶点定位结果三维视图
% (d) 多焊球顶点分布
% 输入: X.mat, Y.mat, Z.mat, hanqiuMask.mat

clear; clc; close all;

%% 1. 数据加载与预处理
try
    load('X.mat'); load('Y.mat'); load('Z.mat'); load('I_masked.mat')
    load('hanqiuMask.mat');
catch
    error('无法加载输入文件 (X.mat, Y.mat, Z.mat, I_masked.mat, hanqiuMask.mat)。');
end
Z =160-Z;
% 预处理掩膜
hanqiuMask = logical(hanqiuMask);
I_masked = logical(I_masked);

% 腐蚀焊球掩膜（用于后续焊球分割）
se_erode = strel('disk', 2);
hanqiuMask_eroded = imerode(hanqiuMask, se_erode);

% 膨胀焊球掩膜（用于基板平面拟合，确保完全排除焊球区域）
se_dilate = strel('disk', 5);
hanqiuMask_dilated = imdilate(hanqiuMask, se_dilate);

%% 2. 基板曲面拟合（考虑翘曲）
validDataMask = (Z ~= 0) & ~isnan(Z);
% 基板掩膜 = 芯片区域 ∩ 非焊球膨胀区域 ∩ 有效数据
substrateMask = I_masked & ~hanqiuMask_dilated & validDataMask;
idx_sub = find(substrateMask);
if isempty(idx_sub), error('未检测到基板区域'); end

% 2.1 提取基板点云（使用全部点）
pts_sub_init = [X(idx_sub), Y(idx_sub), Z(idx_sub)];
pts_sub_init = pts_sub_init(~any(isnan(pts_sub_init),2), :);

fprintf('基板区域点云数量: %d\n', size(pts_sub_init, 1));

% 2.2 初步平面拟合（用于法向量计算）
M_plane = [pts_sub_init(:,1), pts_sub_init(:,2), ones(size(pts_sub_init,1),1)];
B_plane = M_plane \ pts_sub_init(:,3);
% 基板平面法向量 (A, B, -1)，形式: AX + BY - Z + C = 0
A_ref = -B_plane(1);
B_ref = -B_plane(2);
n_ref = [A_ref, B_ref, -1];
n_ref_normalized = n_ref / norm(n_ref);
if n_ref_normalized(3) > 0, n_ref_normalized = -n_ref_normalized; end % 指向下方

% 2.3 基于平面残差的离群点快速筛选（替代逐点法向量计算）
% 利用已拟合的初步平面，计算每个点到平面的残差，剔除偏差过大的点
Z_plane_pred = [pts_sub_init(:,1), pts_sub_init(:,2), ones(size(pts_sub_init,1),1)] * B_plane;
residuals_sub = abs(pts_sub_init(:,3) - Z_plane_pred);
sigma_sub = std(residuals_sub);
residual_thr = 2.5 * sigma_sub;  % 2.5σ阈值
valid_sub_indices = residuals_sub <= residual_thr;
pts_sub_filtered = pts_sub_init(valid_sub_indices, :);

fprintf('基板点云残差筛选: %d -> %d (%.1f%%), 阈值=%.4f mm\n', size(pts_sub_init,1), ...
    size(pts_sub_filtered,1), 100*size(pts_sub_filtered,1)/size(pts_sub_init,1), residual_thr);

% 2.5 多项式曲面拟合（二阶，考虑翘曲）
% Z = a0 + a1*X + a2*Y + a3*X^2 + a4*Y^2 + a5*XY
X_sub = pts_sub_filtered(:,1);
Y_sub = pts_sub_filtered(:,2);
Z_sub = pts_sub_filtered(:,3);
M_surface = [ones(size(X_sub)), X_sub, Y_sub, X_sub.^2, Y_sub.^2, X_sub.*Y_sub];
params_substrate = M_surface \ Z_sub;

% 保存曲面参数用于后续距离计算
substrate_surface_params = params_substrate;

% 更新基板平面参数（使用平面部分用于法向量约束）
B_sub = [params_substrate(2); params_substrate(3); params_substrate(1)];

%% 3. 选择典型焊球进行曲面拟合
CC = bwconncomp(hanqiuMask_eroded);
stats = regionprops(CC, 'Area', 'PixelIdxList');
areas = [stats.Area];
[~, sortIdx] = sort(areas, 'descend');
targetIdx = 30; % 与pic5_06/pic5_07一致

ball_pixels = stats(targetIdx).PixelIdxList;
X_ball = X(ball_pixels);
Y_ball = Y(ball_pixels);
Z_ball = Z(ball_pixels);

% 剔除无效点
valid_ball = ~isnan(X_ball) & ~isnan(Y_ball) & ~isnan(Z_ball) & (Z_ball ~= 0);
points = [X_ball(valid_ball), Y_ball(valid_ball), Z_ball(valid_ball)];

% 法向量约束筛选 (复用pic5_07逻辑)
k_neighbors = 20;
KDTree = KDTreeSearcher(points);
normals = zeros(size(points, 1), 3);
for i = 1:size(points, 1)
    [idx_nbr, ~] = knnsearch(KDTree, points(i,:), 'K', k_neighbors);
    nbr_points = points(idx_nbr, :);
    coeff = pca(nbr_points);
    normal = coeff(:, 3)';
    if dot(normal, n_ref_normalized) > 0, normal = -normal; end
    normals(i, :) = normal;
end

dot_prods = abs(normals * n_ref_normalized');
dot_prods = max(min(dot_prods, 1), 0);
thetas = acos(dot_prods) * 180 / pi;
theta_thr = 45;
valid_indices = thetas <= theta_thr;
points_filtered = points(valid_indices, :);

% 迭代曲面拟合 (简化版，只取最终结果)
max_iter = 100;
points_current = points_filtered;
for iter = 1:max_iter
    X_fit = points_current(:,1);
    Y_fit = points_current(:,2);
    Z_fit = points_current(:,3);
    
    M_fit = [X_fit.^2, Y_fit.^2, X_fit.*Y_fit, X_fit, Y_fit, ones(size(X_fit))];
    params = M_fit \ Z_fit;
    
    Z_pred = M_fit * params;
    residuals = abs(Z_fit - Z_pred);
    
    if iter < max_iter
        sigma_r = std(residuals);
        threshold = 2.5 * sigma_r;
        inliers = residuals < threshold;
        
        if sum(~inliers) == 0 || sum(inliers) < 50
            break;
        end
        points_current = points_current(inliers, :);
    end
end

% 最终曲面参数: Z = aX^2 + bY^2 + cXY + dX + eY + f
a = params(1); b = params(2); c = params(3);
d = params(4); e = params(5); f = params(6);

%% 4. 在焊球曲面上寻找相对基板曲面距离最大的点作为顶点（优化问题）
% 关键思想：既然拟合了曲面，就应该在曲面上求最优解，而不是在离散点中搜索
% 
% 优化目标：max_{X,Y} [Z_ball(X,Y) - Z_substrate(X,Y)]
% 
% 其中：
%   Z_ball = a*X² + b*Y² + c*XY + d*X + e*Y + f
%   Z_substrate = s0 + s1*X + s2*Y + s3*X² + s4*Y² + s5*XY
% 
% 求解条件（偏导数=0）：
%   ∂/∂X [Z_ball - Z_substrate] = 0
%   ∂/∂Y [Z_ball - Z_substrate] = 0
% 
% 展开：
%   (2a*X + c*Y + d) - (s1 + 2*s3*X + s5*Y) = 0
%   (2b*Y + c*X + e) - (s2 + 2*s4*Y + s5*X) = 0
% 
% 整理成线性方程组：
%   (2a - 2*s3)*X + (c - s5)*Y = s1 - d
%   (c - s5)*X + (2b - 2*s4)*Y = s2 - e

% 4.1 构建系数矩阵
s1 = substrate_surface_params(2);  % X的一阶系数
s2 = substrate_surface_params(3);  % Y的一阶系数
s3 = substrate_surface_params(4);  % X²的系数
s4 = substrate_surface_params(5);  % Y²的系数
s5 = substrate_surface_params(6);  % XY的系数

coeff_matrix_dist = [2*a - 2*s3, c - s5;
                     c - s5, 2*b - 2*s4];
rhs_dist = [s1 - d; s2 - e];

% 4.2 求解最优顶点坐标
if abs(det(coeff_matrix_dist)) > 1e-10
    vertex_xy = coeff_matrix_dist \ rhs_dist;
    X_t = vertex_xy(1);
    Y_t = vertex_xy(2);
    
    % 计算顶点Z坐标（在焊球曲面上）
    Z_t = a*X_t^2 + b*Y_t^2 + c*X_t*Y_t + d*X_t + e*Y_t + f;
    
    % 计算对应的基板曲面高度
    z_base_t = substrate_surface_params(1) + s1*X_t + s2*Y_t + s3*X_t^2 + s4*Y_t^2 + s5*X_t*Y_t;
    max_distance = Z_t - z_base_t;
    
    % ===== 关键检查：验证驻点是最大值而非最小值或鞍点 =====
    % 计算Hessian矩阵: H = [∂²h/∂X², ∂²h/∂X∂Y; ∂²h/∂Y∂X, ∂²h/∂Y²]
    % 其中 h(X,Y) = Z_ball(X,Y) - Z_substrate(X,Y)
    H11 = 2*(a - s3);  % ∂²h/∂X²
    H22 = 2*(b - s4);  % ∂²h/∂Y²
    H12 = c - s5;      % ∂²h/∂X∂Y
    
    det_H = H11*H22 - H12^2;
    trace_H = H11 + H22;
    
    % 判断驻点类型：
    % - det(H) > 0 且 trace(H) < 0 → 局部最大值 ✓
    % - det(H) > 0 且 trace(H) > 0 → 局部最小值 ✗
    % - det(H) < 0 → 鞍点 ✗
    is_maximum = (det_H > 1e-6) && (trace_H < -1e-6);
    
    % 验证顶点是否在焊球掩膜区域内（使用像素坐标精确判断）
    % 将物理坐标转换回像素坐标
    diff_X = abs(X - X_t);
    diff_Y = abs(Y - Y_t);
    diff_total = diff_X + diff_Y;
    [~, idx_closest] = min(diff_total(:));
    [row_t, col_t] = ind2sub(size(X), idx_closest);
    
    % 检查该像素是否在当前焊球的掩膜区域内
    in_region = ismember(idx_closest, ball_pixels);
    
    if is_maximum && in_region
        % 曲面极值点是最大值且在焊球区域内，使用它
        vertex = [X_t, Y_t, Z_t];
        fprintf('典型焊球顶点: (%.4f, %.4f, %.4f) mm, 相对基板高度: %.4f mm [✓ 曲面极值点-最大值]\n', ...
                X_t, Y_t, Z_t, max_distance);
    elseif ~is_maximum
        % 驻点不是最大值（可能是最小值或鞍点）
        if trace_H > 0
            warning('曲面驻点是最小值点(trace(H)=%.4f > 0)，回退到点云搜索', trace_H);
        else
            warning('曲面驻点是鞍点(det(H)=%.4f < 0)，回退到点云搜索', det_H);
        end
        num_points = size(points_current, 1);
        distances_to_base = zeros(num_points, 1);
        for i = 1:num_points
            x_i = points_current(i, 1);
            y_i = points_current(i, 2);
            z_i = points_current(i, 3);
            z_base_i = substrate_surface_params(1) + s1*x_i + s2*y_i + s3*x_i^2 + s4*y_i^2 + s5*x_i*y_i;
            distances_to_base(i) = z_i - z_base_i;
        end
        [max_distance, max_idx] = max(distances_to_base);
        X_t = points_current(max_idx, 1);
        Y_t = points_current(max_idx, 2);
        Z_t = points_current(max_idx, 3);
        vertex = [X_t, Y_t, Z_t];
        fprintf('典型焊球顶点: (%.4f, %.4f, %.4f) mm, 相对基板高度: %.4f mm [✓ 点云搜索结果]\n', ...
                X_t, Y_t, Z_t, max_distance);
    else
        % 极值点对应的像素不在焊球掩膜内，回退到点云搜索
        warning('曲面极值点偏离焊球掩膜区域，回退到点云搜索');
        num_points = size(points_current, 1);
        distances_to_base = zeros(num_points, 1);
        for i = 1:num_points
            x_i = points_current(i, 1);
            y_i = points_current(i, 2);
            z_i = points_current(i, 3);
            z_base_i = substrate_surface_params(1) + s1*x_i + s2*y_i + s3*x_i^2 + s4*y_i^2 + s5*x_i*y_i;
            distances_to_base(i) = z_i - z_base_i;
        end
        [max_distance, max_idx] = max(distances_to_base);
        X_t = points_current(max_idx, 1);
        Y_t = points_current(max_idx, 2);
        Z_t = points_current(max_idx, 3);
        vertex = [X_t, Y_t, Z_t];
        fprintf('典型焊球顶点: (%.4f, %.4f, %.4f) mm, 相对基板高度: %.4f mm [✓ 点云搜索结果]\n', ...
                X_t, Y_t, Z_t, max_distance);
    end
else
    warning('系数矩阵奇异（两曲面平行或曲率相同），回退到点云搜索');
    % 回退方案：在点云中找相对基板距离最大的点
    num_points = size(points_current, 1);
    distances_to_base = zeros(num_points, 1);
    for i = 1:num_points
        x_i = points_current(i, 1);
        y_i = points_current(i, 2);
        z_i = points_current(i, 3);
        z_base_i = substrate_surface_params(1) + s1*x_i + s2*y_i + s3*x_i^2 + s4*y_i^2 + s5*x_i*y_i;
        distances_to_base(i) = z_i - z_base_i;
    end
    [max_distance, max_idx] = max(distances_to_base);
    X_t = points_current(max_idx, 1);
    Y_t = points_current(max_idx, 2);
    Z_t = points_current(max_idx, 3);
    vertex = [X_t, Y_t, Z_t];
    fprintf('典型焊球顶点: (%.4f, %.4f, %.4f) mm, 相对基板高度: %.4f mm [✓ 点云搜索结果]\n', ...
            X_t, Y_t, Z_t, max_distance);
end

%% 5. 计算多焊球顶点（用于subplot d）并计算到基板曲面的距离
num_balls_to_show = CC.NumObjects;
vertices_multi = zeros(num_balls_to_show, 3);
distances_to_substrate = zeros(num_balls_to_show, 1);
fallback_flags = false(num_balls_to_show, 1);  % 记录是否使用回退方案

for i = 1:num_balls_to_show
    try
        ball_pixels_i = stats(sortIdx(i)).PixelIdxList;
        pts_i = [X(ball_pixels_i), Y(ball_pixels_i), Z(ball_pixels_i)];
        pts_i = pts_i(~any(isnan(pts_i),2) & pts_i(:,3)~=0, :);
        
        if size(pts_i, 1) < 50, continue; end
        
        % 完整处理流程：法向量约束筛选 + 迭代曲面拟合
        % 1. 法向量约束筛选
        k_neighbors_i = 20;
        KDTree_i = KDTreeSearcher(pts_i);
        normals_i = zeros(size(pts_i, 1), 3);
        for j = 1:size(pts_i, 1)
            [idx_nbr_i, ~] = knnsearch(KDTree_i, pts_i(j,:), 'K', min(k_neighbors_i, size(pts_i,1)));
            nbr_points_i = pts_i(idx_nbr_i, :);
            coeff_i = pca(nbr_points_i);
            normal_i = coeff_i(:, 3)';
            if dot(normal_i, n_ref_normalized) > 0, normal_i = -normal_i; end
            normals_i(j, :) = normal_i;
        end
        
        dot_prods_i = abs(normals_i * n_ref_normalized');
        dot_prods_i = max(min(dot_prods_i, 1), 0);
        thetas_i = acos(dot_prods_i) * 180 / pi;
        theta_thr_i = 45;
        valid_indices_i = thetas_i <= theta_thr_i;
        pts_i_filtered = pts_i(valid_indices_i, :);
        
        if size(pts_i_filtered, 1) < 50, continue; end
        
        % 2. 迭代曲面拟合（RANSAC）- 用于后续可视化
        max_iter_i = 100;
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
                threshold_i = 2.5 * sigma_r_i;
                inliers_i = residuals_i < threshold_i;
                
                if sum(~inliers_i) == 0 || sum(inliers_i) < 50
                    break;
                end
                pts_i_current = pts_i_current(inliers_i, :);
            end
        end
        
        % 3. 在拟合的焊球曲面上求解相对基板距离最大点（优化方法）
        a_i = p_i(1); b_i = p_i(2); c_i = p_i(3);
        d_i = p_i(4); e_i = p_i(5); f_i = p_i(6);
        
        % 构建优化方程系数矩阵
        coeff_matrix_i = [2*a_i - 2*s3, c_i - s5;
                         c_i - s5, 2*b_i - 2*s4];
        rhs_i = [s1 - d_i; s2 - e_i];
        
        % 求解极值点
        if abs(det(coeff_matrix_i)) > 1e-10
            vertex_xy_i = coeff_matrix_i \ rhs_i;
            x_vertex_i = vertex_xy_i(1);
            y_vertex_i = vertex_xy_i(2);
            z_vertex_i = a_i*x_vertex_i^2 + b_i*y_vertex_i^2 + c_i*x_vertex_i*y_vertex_i + ...
                        d_i*x_vertex_i + e_i*y_vertex_i + f_i;
            
            % 计算相对基板高度
            z_base_vertex_i = substrate_surface_params(1) + s1*x_vertex_i + s2*y_vertex_i + ...
                             s3*x_vertex_i^2 + s4*y_vertex_i^2 + s5*x_vertex_i*y_vertex_i;
            max_dist_i = z_vertex_i - z_base_vertex_i;
            
            % 验证Hessian矩阵：确保是最大值点
            H11_i = 2*(a_i - s3);
            H22_i = 2*(b_i - s4);
            H12_i = c_i - s5;
            det_H_i = H11_i*H22_i - H12_i^2;
            trace_H_i = H11_i + H22_i;
            is_maximum_i = (det_H_i > 1e-6) && (trace_H_i < -1e-6);
            
            % 验证是否在焊球掩膜区域内（使用像素坐标精确判断）
            % 将物理坐标转换回像素坐标
            diff_X_i = abs(X - x_vertex_i);
            diff_Y_i = abs(Y - y_vertex_i);
            diff_total_i = diff_X_i + diff_Y_i;
            [~, idx_closest_i] = min(diff_total_i(:));
            
            % 检查该像素是否在当前焊球的掩膜区域内
            in_region_i = ismember(idx_closest_i, ball_pixels_i);
            
            if is_maximum_i && in_region_i
                % 曲面极值点是最大值且在区域内，使用它
                vertices_multi(i, :) = [x_vertex_i, y_vertex_i, z_vertex_i];
                fallback_flags(i) = false;  % 未使用回退
            elseif ~is_maximum_i
                % 驻点不是最大值（是最小值或鞍点）
                if trace_H_i > 0
                    warning('焊球#%d: 曲面驻点是最小值点，回退到点云搜索', i);
                else
                    warning('焊球#%d: 曲面驻点是鞍点，回退到点云搜索', i);
                end
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
                vertices_multi(i, :) = pts_i_current(max_idx_i, :);
                fallback_flags(i) = true;  % 标记使用了回退
            else
                % 极值点对应的像素不在焊球掩膜内，回退到点云搜索
                warning('焊球#%d: 曲面极值点偏离焊球掩膜区域，回退到点云搜索', i);
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
                vertices_multi(i, :) = pts_i_current(max_idx_i, :);
                fallback_flags(i) = true;  % 标记使用了回退
            end
        else
            % 回退到点云搜索
            warning('焊球#%d: 系数矩阵奇异，回退到点云搜索', i);
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
            vertices_multi(i, :) = pts_i_current(max_idx_i, :);
            fallback_flags(i) = true;  % 标记使用了回退
        end
        
        % 4. 存储该顶点到基板曲面的距离
        distances_to_substrate(i) = max_dist_i;
    catch
        continue;
    end
end
% 清理无效数据并同步fallback_flags
valid_mask = vertices_multi(:,3)~=0;
vertices_multi = vertices_multi(valid_mask, :);
distances_to_substrate = distances_to_substrate(valid_mask);
fallback_flags = fallback_flags(valid_mask);

num_fallback = sum(fallback_flags);
num_surface = sum(~fallback_flags);

fprintf('成功计算 %d 个焊球顶点，平均高度: %.4f mm, 标准差: %.4f mm\n', ...
    length(distances_to_substrate), mean(distances_to_substrate), std(distances_to_substrate));
fprintf('  - 曲面极值点方法: %d 个 (%.1f%%)\n', num_surface, 100*num_surface/length(fallback_flags));
fprintf('  - 点云搜索回退: %d 个 (%.1f%%)\n', num_fallback, 100*num_fallback/length(fallback_flags));

%% 6. 绘图
fig = figure('Position', [100, 100, 1200, 800], 'Color', 'w');

% (a) 焊球曲面与基板平面空间关系
subplot(2, 2, 1);
% 绘制焊球点云
scatter3(points_current(:,1), points_current(:,2), points_current(:,3), 5, 'b', 'filled');
hold on;

% 绘制拟合曲面
x_range = linspace(min(points_current(:,1)), max(points_current(:,1)), 20);
y_range = linspace(min(points_current(:,2)), max(points_current(:,2)), 20);
[XX, YY] = meshgrid(x_range, y_range);
ZZ = a*XX.^2 + b*YY.^2 + c*XX.*YY + d*XX + e*YY + f;
surf(XX, YY, ZZ, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', 'r');

% 绘制基板曲面（多项式拟合结果）
x_base = [min(points_current(:,1)), max(points_current(:,1))];
y_base = [min(points_current(:,2)), max(points_current(:,2))];
[XX_base, YY_base] = meshgrid(x_base, y_base);
ZZ_base = substrate_surface_params(1) + ...
          substrate_surface_params(2)*XX_base + ...
          substrate_surface_params(3)*YY_base + ...
          substrate_surface_params(4)*XX_base.^2 + ...
          substrate_surface_params(5)*YY_base.^2 + ...
          substrate_surface_params(6)*XX_base.*YY_base;
surf(XX_base, YY_base, ZZ_base, 'FaceAlpha', 0.2, 'EdgeColor', 'k', 'FaceColor', 'g');

% 标记顶点
plot3(vertex(1), vertex(2), vertex(3), 'mp', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'LineWidth', 2);

set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.3);
xlabel('X/mm', 'FontSize', 10); ylabel('Y/mm', 'FontSize', 10); zlabel('Z/mm', 'FontSize', 10);
% title('(a) 焊球曲面与基板曲面空间关系', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend('焊球点云', '拟合曲面', '基板曲面', '顶点位置', 'Location', 'best');
axis equal; grid on; axis vis3d; view(3);

% (b) 法向量平行约束求解过程
subplot(2, 2, 2);
% 绘制曲面（俯视图为主）
surf(XX, YY, ZZ, 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'FaceColor', 'c');
hold on;

% 在顶点位置绘制曲面法向量
n_surface_at_vertex = [2*a*X_t + c*Y_t + d, 2*b*Y_t + c*X_t + e, -1];
n_surface_normalized = n_surface_at_vertex / norm(n_surface_at_vertex);
quiver3(X_t, Y_t, Z_t, n_surface_normalized(1)*0.3, n_surface_normalized(2)*0.3, ...
    n_surface_normalized(3)*0.3, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.5);

% 绘制基板法向量（从顶点位置）
quiver3(X_t, Y_t, Z_t, n_ref_normalized(1)*0.3, n_ref_normalized(2)*0.3, ...
    n_ref_normalized(3)*0.3, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.5);

% 标记顶点
plot3(X_t, Y_t, Z_t, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'LineWidth', 2);

% 添加顶点位置信息文本
vertex_text = sprintf('顶点(相对基板距离最大点):\nX=%.4f\nY=%.4f\nZ=%.4f\n距离=%.4f mm', X_t, Y_t, Z_t, max_distance);
% text(X_t+0.1, Y_t+0.1, Z_t+0.1, vertex_text, 'FontSize', 8, 'BackgroundColor', 'w', 'EdgeColor', 'k');

set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.3);
xlabel('X/mm', 'FontSize', 10); ylabel('Y/mm', 'FontSize', 10); zlabel('Z/mm', 'FontSize', 10);
% title('(b) 焊球曲面拟合与顶点求解', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend('拟合曲面', '曲面法向量', '基板法向量', '顶点(极值点)', 'Location', 'best');
axis equal; grid on; axis vis3d; view(3);

% (c) 顶点定位结果三维视图
subplot(2, 2, 3);
% 绘制焊球点云
scatter3(points_current(:,1), points_current(:,2), points_current(:,3), 10, 'b', 'filled');
hold on;

% 绘制曲面
surf(XX, YY, ZZ, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', 'r');

% 高亮顶点（相对基板距离最大点）
plot3(X_t, Y_t, Z_t, 'kp', 'MarkerSize', 20, 'MarkerFaceColor', 'y', 'LineWidth', 3);

% 计算点云边界（无论是否使用凸包都需要）
x_min_ball = min(points_current(:,1)); x_max_ball = max(points_current(:,1));
y_min_ball = min(points_current(:,2)); y_max_ball = max(points_current(:,2));
z_min_ball = min(points_current(:,3)); z_max_ball = max(points_current(:,3));

% 绘制焊球点云的实际边界（使用凸包投影到XY平面）
try
    % 在XY平面上计算凸包
    if size(points_current, 1) >= 3
        K = convhull(points_current(:,1), points_current(:,2));
        z_top = max(points_current(:,3));
        z_bottom = min(points_current(:,3));
        
        % 绘制顶部和底部边界
        plot3(points_current(K,1), points_current(K,2), repmat(z_top, length(K), 1), ...
              'm-', 'LineWidth', 1.5);
        plot3(points_current(K,1), points_current(K,2), repmat(z_bottom, length(K), 1), ...
              'm-', 'LineWidth', 1.5);
        
        % 绘制4个代表性的垂直线
        [~, idx_corners] = maxk(vecnorm(points_current(:,1:2) - mean(points_current(:,1:2)), 2, 2), 4);
        for i = 1:length(idx_corners)
            plot3([points_current(idx_corners(i),1) points_current(idx_corners(i),1)], ...
                  [points_current(idx_corners(i),2) points_current(idx_corners(i),2)], ...
                  [z_bottom z_top], 'm--', 'LineWidth', 1);
        end
    end
catch
    % 如果凸包计算失败，回退到矩形包络
    box_x = [x_min_ball x_max_ball x_max_ball x_min_ball x_min_ball];
    box_y = [y_min_ball y_min_ball y_max_ball y_max_ball y_min_ball];
    plot3(box_x, box_y, repmat(z_max_ball, 1, 5), 'm--', 'LineWidth', 1.5);
    plot3(box_x, box_y, repmat(z_min_ball, 1, 5), 'm--', 'LineWidth', 1.5);
    for i = 1:4
        plot3([box_x(i) box_x(i)], [box_y(i) box_y(i)], [z_min_ball z_max_ball], 'm--', 'LineWidth', 1);
    end
end

% 验证顶点是否在区域内
in_region = (X_t >= x_min_ball && X_t <= x_max_ball && ...
             Y_t >= y_min_ball && Y_t <= y_max_ball && ...
             Z_t >= z_min_ball && Z_t <= z_max_ball);
region_status = '✓ 在焊球区域内';
if ~in_region
    region_status = '✗ 偏离焊球区域！';
end

% 从顶点向基板曲面画垂线（示意）
z_base_at_vertex = substrate_surface_params(1) + ...
                   substrate_surface_params(2)*X_t + ...
                   substrate_surface_params(3)*Y_t + ...
                   substrate_surface_params(4)*X_t^2 + ...
                   substrate_surface_params(5)*Y_t^2 + ...
                   substrate_surface_params(6)*X_t*Y_t;
plot3([X_t, X_t], [Y_t, Y_t], [Z_t, z_base_at_vertex], 'g--', 'LineWidth', 2);
plot3(X_t, Y_t, z_base_at_vertex, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

height_single = Z_t - z_base_at_vertex;

set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.3);
xlabel('X/mm', 'FontSize', 10); ylabel('Y/mm', 'FontSize', 10); zlabel('Z/mm', 'FontSize', 10);
% title(sprintf('(c) 顶点定位结果 (相对基板高度=%.4f mm) %s', height_single, region_status), ...
%     'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend('焊球点云', '拟合曲面', '顶点(曲面极值)', '到基板距离', '基板投影', '点云边界', 'Location', 'best');
axis equal; grid on; axis vis3d; view(3);

% (d) 多焊球顶点分布及高度差异
subplot(2, 2, 4);
if ~isempty(vertices_multi)
    % 使用高度值映射颜色
    scatter3(vertices_multi(:,1), vertices_multi(:,2), vertices_multi(:,3), 80, ...
             distances_to_substrate, 'filled', 'MarkerEdgeColor', 'k');
    hold on;
    colorbar;
    caxis([min(distances_to_substrate), max(distances_to_substrate)]);
    colormap(gca, jet);
    
    % 绘制基板曲面（大范围）
    x_v_range = linspace(min(vertices_multi(:,1)), max(vertices_multi(:,1)), 30);
    y_v_range = linspace(min(vertices_multi(:,2)), max(vertices_multi(:,2)), 30);
    [XX_v, YY_v] = meshgrid(x_v_range, y_v_range);
    ZZ_v = substrate_surface_params(1) + ...
           substrate_surface_params(2)*XX_v + ...
           substrate_surface_params(3)*YY_v + ...
           substrate_surface_params(4)*XX_v.^2 + ...
           substrate_surface_params(5)*YY_v.^2 + ...
           substrate_surface_params(6)*XX_v.*YY_v;
    surf(XX_v, YY_v, ZZ_v, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);
    
    % 标注典型焊球顶点
    plot3(X_t, Y_t, Z_t, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    % 添加统计信息文本
    stats_text = sprintf('N=%d\n平均高度: %.4f mm\n标准差: %.4f mm\n最大: %.4f mm\n最小: %.4f mm', ...
                        size(vertices_multi,1), mean(distances_to_substrate), std(distances_to_substrate), ...
                        max(distances_to_substrate), min(distances_to_substrate));
    % text(min(vertices_multi(:,1)), max(vertices_multi(:,2)), max(vertices_multi(:,3)), ...
    %      stats_text, 'FontSize', 8, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'VerticalAlignment', 'top');
    
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.3);
    xlabel('X/mm', 'FontSize', 10); ylabel('Y/mm', 'FontSize', 10); zlabel('Z/mm', 'FontSize', 10);
    % title('(d) 多焊球顶点高度分布', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
    legend('焊球顶点(按高度着色)', '基板曲面', '典型焊球', 'Location', 'best');
    axis equal; grid on; axis vis3d; view(3);
end

sgtitle('图5-8 基于曲面优化的焊球顶点定位', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

%% 7. 绘制焊球高度分布图
fig2 = figure('Position', [150, 150, 1400, 600], 'Color', 'w');

% 左图：条形图显示每个焊球的高度
subplot(1, 2, 1);
bar(1:length(distances_to_substrate), distances_to_substrate, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
hold on;
% 添加平均线
mean_height = mean(distances_to_substrate);
plot([0, length(distances_to_substrate)+1], [mean_height, mean_height], 'r--', 'LineWidth', 2);
% 添加±3σ线
std_height = std(distances_to_substrate);
plot([0, length(distances_to_substrate)+1], [mean_height+3*std_height, mean_height+3*std_height], 'g--', 'LineWidth', 1.5);
plot([0, length(distances_to_substrate)+1], [mean_height-3*std_height, mean_height-3*std_height], 'g--', 'LineWidth', 1.5);

set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.3);
xlabel('焊球编号', 'FontSize', 12, 'FontName', 'SimHei');
ylabel('到基板曲面的高度 (mm)', 'FontSize', 12, 'FontName', 'SimHei');
title('焊球顶点高度分布', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend('焊球高度', sprintf('平均值 (%.4f mm)', mean_height), '±3σ界限', 'Location', 'best');
grid on;
xlim([0, length(distances_to_substrate)+1]);

% 右图：XY平面上的焊球位置热力图
subplot(1, 2, 2);
% 使用散点图，颜色映射高度
scatter(vertices_multi(:,1), vertices_multi(:,2), 200, distances_to_substrate, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
hold on;
colorbar;
colormap(gca, jet);
caxis([min(distances_to_substrate), max(distances_to_substrate)]);

% 添加数值标注
for i = 1:size(vertices_multi, 1)
    text(vertices_multi(i,1), vertices_multi(i,2), sprintf('%.3f', distances_to_substrate(i)), ...
         'FontSize', 7, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
end

% 标注典型焊球
idx_typical = find(abs(vertices_multi(:,1) - X_t) < 0.01 & abs(vertices_multi(:,2) - Y_t) < 0.01, 1);
if ~isempty(idx_typical)
    plot(vertices_multi(idx_typical,1), vertices_multi(idx_typical,2), 'rp', 'MarkerSize', 20, 'LineWidth', 3);
end

set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.3);
xlabel('X/mm', 'FontSize', 12); ylabel('Y/mm', 'FontSize', 12);
title('焊球高度在XY平面的分布', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on;

sgtitle(sprintf('焊球高度统计分析 (N=%d, 均值=%.4f mm, 标准差=%.4f mm)', ...
        length(distances_to_substrate), mean_height, std_height), ...
        'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

%% 8. 绘制原始圆心与重新确认顶点的对比图
fig3 = figure('Position', [200, 200, 1400, 800], 'Color', 'w');

% 加载原始圆心数据
try
    load('ballCircles.mat');
    has_ballCircles = true;
catch
    has_ballCircles = false;
    warning('未找到 ballCircles.mat，无法显示原始圆心');
end

% 加载原始图像用于背景
try
    I_original = imread('bga.bmp');
    if size(I_original, 3) == 3
        I_original = rgb2gray(I_original);
    end
    has_image = true;
catch
    has_image = false;
    warning('未找到原始图像 bga.bmp');
end

if has_image && has_ballCircles
    % 将物理坐标（mm）转换为图像像素坐标
    vertices_pixel = zeros(size(vertices_multi, 1), 2);
    for i = 1:size(vertices_multi, 1)
        % 在X, Y矩阵中找到最接近的像素位置
        diff_X = abs(X - vertices_multi(i, 1));
        diff_Y = abs(Y - vertices_multi(i, 2));
        diff_total = diff_X + diff_Y;
        [~, idx_min] = min(diff_total(:));
        [row, col] = ind2sub(size(X), idx_min);
        vertices_pixel(i, 1) = col;  % x坐标
        vertices_pixel(i, 2) = row;  % y坐标
    end
    
    % 在原图上叠加标记
    imshow(I_original);
    hold on;
    
    % 1. 绘制原始圆心（红色圆圈）
    for i = 1:length(ballCircles)
        xc = ballCircles(i).xc;
        yc = ballCircles(i).yc;
        r = ballCircles(i).r;
        
        % 绘制圆
        theta = linspace(0, 2*pi, 100);
        x_circle = xc + r * cos(theta);
        y_circle = yc + r * sin(theta);
        plot(x_circle, y_circle, 'r-', 'LineWidth', 1.5);
        
        % 标记圆心
        plot(xc, yc, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    end
    
    % 2. 绘制重新确认的顶点（绿色五角星）
    % 使用转换后的图像像素坐标
    for i = 1:size(vertices_pixel, 1)
        x_vertex = vertices_pixel(i, 1);
        y_vertex = vertices_pixel(i, 2);
        
        % 绘制顶点（五角星）
        plot(x_vertex, y_vertex, 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g', ...
             'LineWidth', 2, 'MarkerEdgeColor', 'k');
    end
    
    % 3. 标注典型焊球（编号30）
    if length(ballCircles) >= targetIdx
        xc_target = ballCircles(targetIdx).xc;
        yc_target = ballCircles(targetIdx).yc;
        plot(xc_target, yc_target, 'mo', 'MarkerSize', 15, 'LineWidth', 3);
        text(xc_target+20, yc_target-20, sprintf('典型焊球#%d', targetIdx), ...
             'Color', 'm', 'FontSize', 11, 'FontWeight', 'bold', ...
             'BackgroundColor', [1, 1, 1, 0.8], 'EdgeColor', 'm');
    end
    
    % 添加图例
    h1 = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'LineWidth', 1.5);
    h2 = plot(NaN, NaN, 'gp', 'MarkerFaceColor', 'g', 'MarkerSize', 15, 'LineWidth', 2, 'MarkerEdgeColor', 'k');
    legend([h1, h2], {'原始圆心定位', '曲面优化顶点'}, ...
           'Location', 'best', 'FontSize', 11);
    
    title('原始圆心定位 vs 曲面优化顶点定位对比', ...
          'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
    hold off;
else
    % 如果没有图像或圆心数据，只显示3D顶点分布（使用物理坐标）
    if size(vertices_multi, 1) > 0 
        scatter(vertices_multi(:,1), vertices_multi(:,2), 100, 'g', 'filled', 'MarkerEdgeColor', 'k');
        title('曲面优化顶点位置（物理坐标）', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
        xlabel('X (mm)', 'FontSize', 12); ylabel('Y (mm)', 'FontSize', 12);
        axis equal; grid on;
    end
end

%% 9. 绘制回退焊球验证图（验证点云搜索方法的正确性）
if num_fallback > 0
    fig4 = figure('Position', [250, 250, 1400, 900], 'Color', 'w');
    
    % 找出使用回退的焊球索引
    fallback_indices = find(fallback_flags);
    num_to_show = min(6, num_fallback);  % 最多显示6个
    
    for idx = 1:num_to_show
        fb_idx = fallback_indices(idx);
        
        % 重新提取该焊球的点云数据
        ball_pixels_fb = stats(sortIdx(fb_idx)).PixelIdxList;
        pts_fb = [X(ball_pixels_fb), Y(ball_pixels_fb), Z(ball_pixels_fb)];
        pts_fb = pts_fb(~any(isnan(pts_fb),2) & pts_fb(:,3)~=0, :);
        
        if size(pts_fb, 1) < 50, continue; end
        
        % 绘制子图
        subplot(2, 3, idx);
        
        % 1. 绘制焊球点云
        scatter3(pts_fb(:,1), pts_fb(:,2), pts_fb(:,3), 10, 'b', 'filled', 'MarkerFaceColor', [0.3 0.6 0.9]);
        hold on;
        
        % 2. 获取该焊球的顶点和高度
        vertex_fb = vertices_multi(fb_idx, :);
        dist_fb = distances_to_substrate(fb_idx);
        
        % 3. 绘制顶点（使用橙色五角星标记回退点）
        plot3(vertex_fb(1), vertex_fb(2), vertex_fb(3), 'p', ...
              'MarkerSize', 18, 'MarkerFaceColor', [1 0.5 0], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % 4. 计算并绘制到基板的垂线
        x_fb = vertex_fb(1);
        y_fb = vertex_fb(2);
        z_fb = vertex_fb(3);
        z_base_fb = substrate_surface_params(1) + s1*x_fb + s2*y_fb + ...
                   s3*x_fb^2 + s4*y_fb^2 + s5*x_fb*y_fb;
        plot3([x_fb, x_fb], [y_fb, y_fb], [z_fb, z_base_fb], 'r--', 'LineWidth', 2.5);
        plot3(x_fb, y_fb, z_base_fb, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        % 5. 绘制点云边界（凸包）
        try
            if size(pts_fb, 1) >= 3
                K_fb = convhull(pts_fb(:,1), pts_fb(:,2));
                z_top_fb = max(pts_fb(:,3));
                z_bottom_fb = min(pts_fb(:,3));
                plot3(pts_fb(K_fb,1), pts_fb(K_fb,2), repmat(z_top_fb, length(K_fb), 1), ...
                      'm:', 'LineWidth', 1.5);
                plot3(pts_fb(K_fb,1), pts_fb(K_fb,2), repmat(z_bottom_fb, length(K_fb), 1), ...
                      'm:', 'LineWidth', 1.5);
            end
        catch
            % 回退到矩形框
            x_min_fb = min(pts_fb(:,1)); x_max_fb = max(pts_fb(:,1));
            y_min_fb = min(pts_fb(:,2)); y_max_fb = max(pts_fb(:,2));
            z_min_fb = min(pts_fb(:,3)); z_max_fb = max(pts_fb(:,3));
            
            box_x_fb = [x_min_fb x_max_fb x_max_fb x_min_fb x_min_fb];
            box_y_fb = [y_min_fb y_min_fb y_max_fb y_max_fb y_min_fb];
            plot3(box_x_fb, box_y_fb, repmat(z_max_fb, 1, 5), 'm:', 'LineWidth', 1.5);
            plot3(box_x_fb, box_y_fb, repmat(z_min_fb, 1, 5), 'm:', 'LineWidth', 1.5);
            for i = 1:4
                plot3([box_x_fb(i) box_x_fb(i)], [box_y_fb(i) box_y_fb(i)], ...
                      [z_min_fb z_max_fb], 'm:', 'LineWidth', 1);
            end
        end
        
        % 6. 添加标题和标签
        xlabel('X/mm', 'FontSize', 9); ylabel('Y/mm', 'FontSize', 9); zlabel('Z/mm', 'FontSize', 9);
        title(sprintf('焊球#%d (回退)\n顶点: (%.3f, %.3f, %.3f)\n相对高度: %.4f mm', ...
              fb_idx, x_fb, y_fb, z_fb, dist_fb), ...
              'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimHei');
        
        % 7. 设置视图
        grid on; axis equal; axis vis3d; view(3);
        set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
        
        if idx == 1
            legend('焊球点云', '顶点(点云搜索)', '到基板距离', '基板投影', ...
                   'Location', 'best', 'FontSize', 7);
        end
    end
    
    sgtitle(sprintf('回退焊球验证图 (点云搜索方法) - 共%d个回退，显示前%d个', ...
            num_fallback, num_to_show), ...
            'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
else
    fprintf('\n所有焊球均使用曲面极值点方法，无回退情况。\n');
end

%% 10. 绘制所有焊球顶点分布图（区分曲面极值点和回退点）
fig5 = figure('Position', [300, 300, 800, 700], 'Color', 'w');

% 分离曲面极值点和回退点
vertices_surface = vertices_multi(~fallback_flags, :);
vertices_fallback = vertices_multi(fallback_flags, :);
dist_surface = distances_to_substrate(~fallback_flags);
dist_fallback = distances_to_substrate(fallback_flags);

% 绘制曲面极值点（蓝色）
if ~isempty(vertices_surface)
    scatter3(vertices_surface(:,1), vertices_surface(:,2), vertices_surface(:,3), ...
             100, dist_surface, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    hold on;
end

% 绘制回退点（橙色边框，突出显示）
if ~isempty(vertices_fallback)
    scatter3(vertices_fallback(:,1), vertices_fallback(:,2), vertices_fallback(:,3), ...
             150, dist_fallback, 'p', 'filled', 'MarkerEdgeColor', [1 0.5 0], 'LineWidth', 3);
end

colorbar;
colormap(jet);
caxis([min(distances_to_substrate), max(distances_to_substrate)]);

% 绘制基板曲面
if ~isempty(vertices_multi)
    x_v_range = linspace(min(vertices_multi(:,1)), max(vertices_multi(:,1)), 30);
    y_v_range = linspace(min(vertices_multi(:,2)), max(vertices_multi(:,2)), 30);
    [XX_v, YY_v] = meshgrid(x_v_range, y_v_range);
    ZZ_v = substrate_surface_params(1) + ...
           substrate_surface_params(2)*XX_v + ...
           substrate_surface_params(3)*YY_v + ...
           substrate_surface_params(4)*XX_v.^2 + ...
           substrate_surface_params(5)*YY_v.^2 + ...
           substrate_surface_params(6)*XX_v.*YY_v;
    surf(XX_v, YY_v, ZZ_v, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);
end

xlabel('X/mm', 'FontSize', 11); ylabel('Y/mm', 'FontSize', 11); zlabel('Z/mm', 'FontSize', 11);
title(sprintf('所有焊球顶点分布（按计算方法分类）\n曲面极值: %d个  |  点云回退: %d个', ...
      num_surface, num_fallback), ...
      'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimHei');

if ~isempty(vertices_fallback)
    legend('曲面极值点', '点云搜索回退', '基板曲面', 'Location', 'best', 'FontSize', 10);
else
    legend('曲面极值点', '基板曲面', 'Location', 'best', 'FontSize', 10);
end

grid on; axis equal; axis vis3d; view(3);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');

%% 11. 绘制小于平均高度焊球的详细验证图
mean_height = mean(distances_to_substrate);
below_mean_mask = distances_to_substrate < mean_height;
below_mean_indices = find(below_mean_mask);
num_below_mean = sum(below_mean_mask);

if num_below_mean > 0
    fprintf('\n发现 %d 个低于平均高度(%.4f mm)的焊球\n', num_below_mean, mean_height);
    
    % 设置布局：2行4列，每张图最多8个
    num_rows = 2;
    num_cols = 4;
    balls_per_fig = num_rows * num_cols;  % 每张图8个
    num_figs = ceil(num_below_mean / balls_per_fig);
    
    fig6_handles = [];  % 存储所有figure句柄
    
    for fig_idx = 1:num_figs
        fig6 = figure('Position', [350, 100, 1600, 800], 'Color', 'w');
        fig6_handles = [fig6_handles; fig6];
        
        % 计算当前figure显示的焊球范围
        start_idx = (fig_idx - 1) * balls_per_fig + 1;
        end_idx = min(fig_idx * balls_per_fig, num_below_mean);
        num_in_this_fig = end_idx - start_idx + 1;
        
        for plot_idx = 1:num_in_this_fig
            ball_idx = below_mean_indices(start_idx + plot_idx - 1);
            
            % 重新提取该焊球的点云数据
            ball_pixels_low = stats(sortIdx(ball_idx)).PixelIdxList;
            pts_low = [X(ball_pixels_low), Y(ball_pixels_low), Z(ball_pixels_low)];
            pts_low = pts_low(~any(isnan(pts_low),2) & pts_low(:,3)~=0, :);
            
            if size(pts_low, 1) < 50, continue; end
            
            % 对该焊球重新进行完整处理以获取拟合曲面
            % 1. 法向量约束筛选
            k_neighbors_low = 20;
            KDTree_low = KDTreeSearcher(pts_low);
            normals_low = zeros(size(pts_low, 1), 3);
            for j = 1:size(pts_low, 1)
                [idx_nbr_low, ~] = knnsearch(KDTree_low, pts_low(j,:), 'K', min(k_neighbors_low, size(pts_low,1)));
                nbr_points_low = pts_low(idx_nbr_low, :);
                coeff_low = pca(nbr_points_low);
                normal_low = coeff_low(:, 3)';
                if dot(normal_low, n_ref_normalized) > 0, normal_low = -normal_low; end
                normals_low(j, :) = normal_low;
            end
            
            dot_prods_low = abs(normals_low * n_ref_normalized');
            dot_prods_low = max(min(dot_prods_low, 1), 0);
            thetas_low = acos(dot_prods_low) * 180 / pi;
            theta_thr_low = 45;
            valid_indices_low = thetas_low <= theta_thr_low;
            pts_low_filtered = pts_low(valid_indices_low, :);
            
            if size(pts_low_filtered, 1) < 50, continue; end
            
            % 2. 迭代曲面拟合
            max_iter_low = 100;
            pts_low_current = pts_low_filtered;
            for iter = 1:max_iter_low
                X_low = pts_low_current(:,1);
                Y_low = pts_low_current(:,2);
                Z_low = pts_low_current(:,3);
                
                M_low = [X_low.^2, Y_low.^2, X_low.*Y_low, X_low, Y_low, ones(size(X_low))];
                p_low = M_low \ Z_low;
                
                Z_pred_low = M_low * p_low;
                residuals_low = abs(Z_low - Z_pred_low);
                
                if iter < max_iter_low
                    sigma_r_low = std(residuals_low);
                    threshold_low = 2.5 * sigma_r_low;
                    inliers_low = residuals_low < threshold_low;
                    
                    if sum(~inliers_low) == 0 || sum(inliers_low) < 50
                        break;
                    end
                    pts_low_current = pts_low_current(inliers_low, :);
                end
            end
            
            % 生成拟合曲面网格
            x_range_low = linspace(min(pts_low_current(:,1)), max(pts_low_current(:,1)), 15);
            y_range_low = linspace(min(pts_low_current(:,2)), max(pts_low_current(:,2)), 15);
            [XX_low, YY_low] = meshgrid(x_range_low, y_range_low);
            ZZ_low = p_low(1)*XX_low.^2 + p_low(2)*YY_low.^2 + p_low(3)*XX_low.*YY_low + ...
                     p_low(4)*XX_low + p_low(5)*YY_low + p_low(6);
            
            % 获取该焊球的顶点和高度
            vertex_low = vertices_multi(ball_idx, :);
            dist_low = distances_to_substrate(ball_idx);
            
            % 计算相对于平均高度的偏差
            height_diff = dist_low - mean_height;
            
            % 绘制子图
            subplot(num_rows, num_cols, plot_idx);
            
            % 1. 绘制焊球点云
            scatter3(pts_low_current(:,1), pts_low_current(:,2), pts_low_current(:,3), ...
                     8, 'b', 'filled', 'MarkerFaceColor', [0.4 0.7 1]);
            hold on;
            
            % 2. 绘制拟合曲面
            surf(XX_low, YY_low, ZZ_low, 'FaceAlpha', 0.35, 'EdgeColor', 'none', 'FaceColor', [1 0.5 0.5]);
            
            % 3. 绘制顶点（使用红色菱形标记低于平均高度）
            plot3(vertex_low(1), vertex_low(2), vertex_low(3), 'd', ...
                  'MarkerSize', 16, 'MarkerFaceColor', [1 0.2 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
            
            % 4. 计算并绘制到基板的垂线
            x_v_low = vertex_low(1);
            y_v_low = vertex_low(2);
            z_v_low = vertex_low(3);
            z_base_low = substrate_surface_params(1) + s1*x_v_low + s2*y_v_low + ...
                        s3*x_v_low^2 + s4*y_v_low^2 + s5*x_v_low*y_v_low;
            plot3([x_v_low, x_v_low], [y_v_low, y_v_low], [z_v_low, z_base_low], ...
                  'g--', 'LineWidth', 2);
            
            % 5. 绘制基板投影点
            plot3(x_v_low, y_v_low, z_base_low, 'go', 'MarkerSize', 7, 'MarkerFaceColor', 'g');
            
            % 6. 绘制基板曲面（局部）
            x_base_low = [min(pts_low_current(:,1))-0.2, max(pts_low_current(:,1))+0.2];
            y_base_low = [min(pts_low_current(:,2))-0.2, max(pts_low_current(:,2))+0.2];
            [XX_base_low, YY_base_low] = meshgrid(x_base_low, y_base_low);
            ZZ_base_low = substrate_surface_params(1) + s1*XX_base_low + s2*YY_base_low + ...
                         s3*XX_base_low.^2 + s4*YY_base_low.^2 + s5*XX_base_low.*YY_base_low;
            surf(XX_base_low, YY_base_low, ZZ_base_low, 'FaceAlpha', 0.2, ...
                 'EdgeColor', 'none', 'FaceColor', [0.6 0.9 0.6]);
            
            % 7. 设置标题和标签 - 包含相对高度信息
            xlabel('X/mm', 'FontSize', 7); ylabel('Y/mm', 'FontSize', 7); zlabel('Z/mm', 'FontSize', 7);
            
            % 判断是否为回退计算
            if fallback_flags(ball_idx)
                fallback_str = ' [回退]';
            else
                fallback_str = '';
            end
            
            title(sprintf('焊球#%d%s\n顶点:(%.3f,%.3f,%.3f)\n相对高度:%.4fmm (%.4fmm)', ...
                  ball_idx, fallback_str, x_v_low, y_v_low, z_v_low, dist_low, height_diff), ...
                  'FontSize', 8, 'FontWeight', 'bold', 'FontName', 'SimHei');
            
            % 8. 设置视图
            grid on; axis equal; axis vis3d; view(3);
            set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'FontSize', 7);
            
            if plot_idx == 1
                legend('焊球点云', '拟合曲面', '顶点(低于平均)', '到基板距离', '基板投影', '基板曲面', ...
                       'Location', 'best', 'FontSize', 6);
            end
        end
        
        % 设置整体标题
        if num_figs > 1
            sgtitle(sprintf('小于平均高度焊球详细分析图 (第%d/%d页, 平均高度=%.4f mm, 共%d个)', ...
                    fig_idx, num_figs, mean_height, num_below_mean), ...
                    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
        else
            sgtitle(sprintf('小于平均高度焊球详细分析图 (平均高度=%.4f mm, 共%d个)', ...
                    mean_height, num_below_mean), ...
                    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
        end
    end
else
    fprintf('\n所有焊球高度均≥平均高度，无异常低点。\n');
end

%% 保存
savePath = 'F:\cs_project\3D\光线回溯法-远心标定\远心单目双光栅\交比不变性拟合\交比不变性重建\Chip\BGA\pointCal';
saveas(fig, fullfile(savePath, 'fig5_08_曲面优化顶点定位.png'));
saveas(fig2, fullfile(savePath, 'fig5_08_焊球高度分布统计.png'));
saveas(fig3, fullfile(savePath, 'fig5_08_原始圆心与顶点对比.png'));
if num_fallback > 0
    saveas(fig4, fullfile(savePath, 'fig5_08_回退焊球验证图.png'));
    saveas(fig5, fullfile(savePath, 'fig5_08_顶点分类分布图.png'));
end
if num_below_mean > 0
    % 保存所有低于平均高度的图
    for fig_idx = 1:length(fig6_handles)
        if length(fig6_handles) > 1
            saveas(fig6_handles(fig_idx), fullfile(savePath, sprintf('fig5_08_低于平均高度焊球分析_第%d页.png', fig_idx)));
        else
            saveas(fig6_handles(fig_idx), fullfile(savePath, 'fig5_08_低于平均高度焊球分析.png'));
        end
    end
end
fprintf('图5-8生成完成\n');
fprintf('图片已保存至: %s\n', savePath);
