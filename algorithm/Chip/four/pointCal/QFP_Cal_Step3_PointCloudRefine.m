% fig5_11_3.m - 图5-11-3: 基于法向量约束与空间连通性的点云精化
%
% 功能：对高度筛选后的底部候选点云，基于局部法向量一致性约束和
%       空间连通性分析剔除边缘噪声和离群点
%
% 解决问题：
%   1. 高度筛选后可能包含引脚底部边缘的突变点
%   2. 测量噪声和相位误差产生的离群点需剔除
%   3. 确保保留点形成连续的平面区域
%
% 核心方法：
%   局部法向量计算（PCA）：
%     在r_local邻域内计算协方差矩阵
%     最小特征值特征向量 = 局部法向量
%   法向量一致性约束：
%     计算局部法向量与本体法向量夹角 theta
%     剔除 theta > theta_thr 的异常点
%   空间连通性分析：
%     基于欧氏距离聚类
%     保留主连通簇，剔除孤立点
%
% 输入：
%   pins_bottom_candidates - 来自fig5_11_2的底部候选点云结构体数组
%   body_surface - 来自fig5_11_1的封装本体曲面信息
%   params - 参数结构体（可选）：
%     .r_local - 局部邻域半径（默认0.2 mm）
%     .theta_thr - 法向量夹角阈值（默认30°）
%     .cluster_dist - 聚类距离阈值（默认0.3 mm）
%
% 输出：
%   pins_bottom_refined - 精化后的引脚底部点云结构体数组：
%     [i].pin_id - 引脚ID
%     [i].side - 引脚所属侧
%     [i].point_cloud_bottom_candidate - 高度筛选后的候选点云
%     [i].point_cloud_bottom_refined - 精化后的有效点云
%     [i].outliers - 被剔除的异常点
%     [i].local_normals - 局部法向量（每个候选点）
%     [i].normal_angles - 法向量夹角（每个候选点）
%     [i].n_clusters - 聚类数量
%
% 使用方法：
%   pins_refined = fig5_11_3(pins_bottom, body_surface);
%   或运行测试脚本：test_fig5_11_3

function pins_bottom_refined = QFP_Cal_Step3_PointCloudRefine(pins_bottom_candidates, body_surface, params)

fprintf('====== 图5-11-3: 基于法向量约束与空间连通性的点云精化 ======\n');

%% 1. 参数初始化
if nargin < 3 || isempty(params)
    params = struct();
end
if ~isfield(params, 'r_local'), params.r_local = 0.2; end
if ~isfield(params, 'theta_thr'), params.theta_thr = 30; end
if ~isfield(params, 'cluster_dist'), params.cluster_dist = 0.3; end

fprintf('参数设置:\n');
fprintf('  r_local (局部邻域半径) = %.3f mm\n', params.r_local);
fprintf('  theta_thr (法向量夹角阈值) = %.1f°\n', params.theta_thr);
fprintf('  cluster_dist (聚类距离阈值) = %.3f mm\n', params.cluster_dist);

n_body_vec = body_surface.normal_vector;
fprintf('\n封装本体法向量: [%.4f, %.4f, %.4f]\n', n_body_vec);

%% 2. 遍历各引脚执行精化
N_pins = length(pins_bottom_candidates);
fprintf('\n开始处理 %d 个引脚...\n', N_pins);

pins_bottom_refined = struct('pin_id', {}, 'side', {}, ...
    'point_cloud_bottom_candidate', {}, 'point_cloud_after_radius_filter', {}, ...
    'point_cloud_bottom_refined', {}, ...
    'outliers', {}, 'local_normals', {}, 'normal_angles', {}, 'n_clusters', {});

for i = 1:N_pins
    pin = pins_bottom_candidates(i);
    fprintf('  [%d/%d] 引脚ID=%d, 侧=%s\n', i, N_pins, pin.pin_id, pin.side);
    
    pc_bottom = pin.point_cloud_bottom;
    N_bottom = size(pc_bottom, 1);
    
    if N_bottom < 10
        fprintf('    警告: 候选点数过少(%d)，跳过精化\n', N_bottom);
        continue;
    end
    
    fprintf('    候选点数: %d\n', N_bottom);
    
    % 保存原始候选点云（半径滤波前）
    pc_bottom_orig = pc_bottom;
    
    %% 半径滤波：去除孤立点
    % 对每个点统计半径内的邻居数，去除邻居过少的孤立点
    radius_filter = 0.05;  % mm，搜索半径
    min_neighbors = 5;      % 最少邻居数
    
    neighbor_counts = zeros(N_bottom, 1);
    for j = 1:N_bottom
        dists = sqrt(sum((pc_bottom - pc_bottom(j, :)).^2, 2));
        neighbor_counts(j) = sum(dists <= radius_filter) - 1;  % 排除自己
    end
    
    valid_radius_idx = neighbor_counts >= min_neighbors;
    pc_bottom = pc_bottom(valid_radius_idx, :);
    n_radius_removed = N_bottom - size(pc_bottom, 1);
    
    if n_radius_removed > 0
        fprintf('    半径滤波去除孤立点: %d (半径=%.2fmm, min_neighbors=%d)\n', ...
                n_radius_removed, radius_filter, min_neighbors);
    end
    
    N_bottom = size(pc_bottom, 1);
    if N_bottom < 10
        fprintf('    警告: 滤波后点数过少(%d)，跳过精化\n', N_bottom);
        continue;
    end
    
    %% 2.1 计算局部法向量（PCA）
    local_normals = zeros(N_bottom, 3);
    normal_angles = zeros(N_bottom, 1);
    
    for j = 1:N_bottom
        % 找邻域点
        pt = pc_bottom(j, :);
        dists = sqrt(sum((pc_bottom - pt).^2, 2));
        neighbors_idx = dists <= params.r_local;
        
        if sum(neighbors_idx) < 5
            % 邻域点太少，设为无效
            normal_angles(j) = inf;
            continue;
        end
        
        neighbors = pc_bottom(neighbors_idx, :);
        
        % 计算协方差矩阵
        centroid_local = mean(neighbors, 1);
        centered = neighbors - centroid_local;
        C = (centered' * centered) / size(neighbors, 1);
        
        % 特征值分解
        [V, D] = eig(C);
        [~, min_idx] = min(diag(D));
        n_local = V(:, min_idx)';
        
        % 确保法向量指向上方（与Z轴正向夹角<90°）
        if n_local(3) < 0
            n_local = -n_local;
        end
        
        local_normals(j, :) = n_local;
        
        % 计算与本体法向量的夹角
        cos_angle = abs(dot(n_local, n_body_vec));
        cos_angle = max(min(cos_angle, 1.0), 0.0);  % 防止数值误差
        angle_deg = acosd(cos_angle);
        normal_angles(j) = angle_deg;
    end
    
    %% 2.2 法向量夹角约束筛选
    valid_normal_idx = normal_angles <= params.theta_thr;
    pc_filtered = pc_bottom(valid_normal_idx, :);
    N_filtered = size(pc_filtered, 1);
    
    fprintf('    法向量筛选: %d -> %d 点 (剔除%d)\n', ...
            N_bottom, N_filtered, N_bottom - N_filtered);
    
    if N_filtered < 5
        fprintf('    警告: 筛选后点数过少(%d)，跳过\n', N_filtered);
        continue;
    end
    
    %% 2.3 空间连通性分析（聚类）
    % 使用简单的基于距离的聚类
    cluster_labels = zeros(N_filtered, 1);
    current_cluster = 0;
    
    for j = 1:N_filtered
        if cluster_labels(j) > 0
            continue;  % 已分配
        end
        
        % 开始新簇
        current_cluster = current_cluster + 1;
        cluster_labels(j) = current_cluster;
        
        % 广度优先搜索邻近点
        queue = j;
        while ~isempty(queue)
            current_pt = queue(1);
            queue(1) = [];
            
            % 找邻近未分配点
            dists = sqrt(sum((pc_filtered - pc_filtered(current_pt, :)).^2, 2));
            near_unassigned = find((dists <= params.cluster_dist) & (cluster_labels == 0));
            
            for k = near_unassigned'
                cluster_labels(k) = current_cluster;
                queue = [queue; k];
            end
        end
    end
    
    n_clusters = max(cluster_labels);
    fprintf('    检测到 %d 个簇\n', n_clusters);
    
    % 保留最大簇
    if n_clusters > 0
        cluster_sizes = histcounts(cluster_labels, 1:n_clusters+1);
        [~, largest_cluster] = max(cluster_sizes);
        main_cluster_idx = (cluster_labels == largest_cluster);
        pc_refined = pc_filtered(main_cluster_idx, :);
        
        fprintf('    保留最大簇: %d 点\n', size(pc_refined, 1));
    else
        pc_refined = pc_filtered;
    end
    
    %% 2.4 识别离群点（与原始候选点云比较）
    outliers = setdiff(pc_bottom_orig, pc_refined, 'rows');
    
    fprintf('    最终有效点数: %d (保留率 %.1f%%)\n', ...
            size(pc_refined, 1), 100*size(pc_refined,1)/size(pc_bottom_orig,1));
    
    %% 2.5 保存结果
    result = struct();
    result.pin_id = pin.pin_id;
    result.side = pin.side;
    result.point_cloud_bottom_candidate = pc_bottom_orig;
    result.point_cloud_after_radius_filter = pc_bottom;
    result.point_cloud_bottom_refined = pc_refined;
    result.outliers = outliers;
    result.local_normals = local_normals;
    result.normal_angles = normal_angles;
    result.n_clusters = n_clusters;
    
    pins_bottom_refined(end+1) = result;
end

fprintf('\n成功精化 %d 个引脚\n', length(pins_bottom_refined));

%% 3. 可视化（选取若干典型引脚）
fprintf('\n生成可视化结果（展示前4个引脚）...\n');
visualize_normal_filtering(pins_bottom_refined, body_surface, ...
                           min(4, length(pins_bottom_refined)));

fprintf('====== 法向量约束精化完成 ======\n\n');

end

%% ========== 辅助函数：可视化 ==========
function visualize_normal_filtering(pins_refined, body_surface, n_show)

if isempty(pins_refined)
    fprintf('  无有效引脚可视化\n');
    return;
end

n_show = min(n_show, length(pins_refined));
n_body_vec = body_surface.normal_vector;

figure('Position', [100, 100, 1600, 900], 'Color', 'w', 'Name', '图5-11-3');

for idx = 1:n_show
    pin = pins_refined(idx);
    pc_cand = pin.point_cloud_bottom_candidate;       % 原始候选（半径滤波前）
    pc_after_rf = pin.point_cloud_after_radius_filter; % 半径滤波后（与normals对应）
    pc_refined = pin.point_cloud_bottom_refined;
    outliers = pin.outliers;
    
    %% 子图(a): 高度筛选后的候选点云
    subplot(n_show, 4, (idx-1)*4 + 1);
    scatter3(pc_cand(:,1), pc_cand(:,2), pc_cand(:,3), 20, pc_cand(:,3), 'filled');
    colormap(gca, jet);
    xlabel('X(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    ylabel('Y(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    zlabel('Z(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    if idx == 1
        title('(a) 高度筛选后的候选点云', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
    end
    title_str = sprintf('引脚%d (%s侧)\n候选%d点', pin.pin_id, pin.side, size(pc_cand,1));
    text(mean(pc_cand(:,1)), mean(pc_cand(:,2)), max(pc_cand(:,3)), title_str, ...
         'FontSize', 9, 'FontName', 'SimHei', 'HorizontalAlignment', 'center', ...
         'BackgroundColor', [1 1 1 0.7]);
    axis equal; grid on; view(45, 30);
    
    %% 子图(b): 局部法向量分布可视化
    subplot(n_show, 4, (idx-1)*4 + 2);
    % 绘制半径滤波后的点云（与法向量数组尺寸一致）
    scatter3(pc_after_rf(:,1), pc_after_rf(:,2), pc_after_rf(:,3), 20, pin.normal_angles, 'filled');
    hold on;
    
    % 可视化部分点的法向量（每5个点显示一个）
    step = max(1, floor(size(pc_after_rf,1) / 20));
    for j = 1:step:size(pc_after_rf,1)
        if ~isinf(pin.normal_angles(j))
            pt = pc_after_rf(j, :);
            nv = pin.local_normals(j, :) * 0.1;  % 缩放箭头长度
            quiver3(pt(1), pt(2), pt(3), nv(1), nv(2), nv(3), ...
                    'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
        end
    end
    
    colormap(gca, jet);
    cb = colorbar('FontSize', 9);
    ylabel(cb, '夹角(°)', 'FontSize', 9, 'FontName', 'SimHei');
    xlabel('X(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    ylabel('Y(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    zlabel('Z(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    if idx == 1
        title('(b) 局部法向量分布', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
    end
    axis equal; grid on; view(45, 30);
    
    %% 子图(c): 精化筛选结果
    subplot(n_show, 4, (idx-1)*4 + 3);
    hold on;
    h_legend = [];
    legend_labels = {};
    % 绘制保留的点（绿色）— 先画保留点作为底层
    h_ref = scatter3(pc_refined(:,1), pc_refined(:,2), pc_refined(:,3), 20, 'g', 'filled');
    h_legend(end+1) = h_ref;
    legend_labels{end+1} = sprintf('保留点(%d)', size(pc_refined,1));
    % 绘制被剔除的点（红色）— 后画剔除点叠在上面
    if ~isempty(outliers)
        h_out = scatter3(outliers(:,1), outliers(:,2), outliers(:,3), 30, 'r', 'filled', ...
                 'MarkerFaceAlpha', 0.5);
        h_legend(end+1) = h_out;
        legend_labels{end+1} = sprintf('剔除点(%d)', size(outliers,1));
    end
    
    xlabel('X(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    ylabel('Y(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    zlabel('Z(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    if idx == 1
        title('(c) 精化筛选结果', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
    end
    legend(h_legend, legend_labels, 'Location', 'best', 'FontSize', 8, 'FontName', 'SimHei');
    axis equal; grid on; view(45, 30);
    
    %% 子图(d): 空间连通性分析后的有效点云
    subplot(n_show, 4, (idx-1)*4 + 4);
    % 绘制候选点云（灰色透明）
    scatter3(pc_cand(:,1), pc_cand(:,2), pc_cand(:,3), 15, [0.7 0.7 0.7], ...
             'filled', 'MarkerFaceAlpha', 0.2);
    hold on;
    % 绘制精化后点云（蓝色高亮）
    scatter3(pc_refined(:,1), pc_refined(:,2), pc_refined(:,3), 25, 'b', 'filled');
    
    % 绘制本体法向量参考（从质心）
    centroid = mean(pc_refined, 1);
    arrow_len = 0.3;
    quiver3(centroid(1), centroid(2), centroid(3), ...
            n_body_vec(1)*arrow_len, n_body_vec(2)*arrow_len, n_body_vec(3)*arrow_len, ...
            'm', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
    
    xlabel('X(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    ylabel('Y(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    zlabel('Z(mm)', 'FontSize', 9, 'FontName', 'SimHei');
    if idx == 1
        title('(d) 空间连通性分析后的有效点云', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
    end
    legend({'候选点', '有效点', '本体法向'}, 'Location', 'best', 'FontSize', 8, 'FontName', 'SimHei');
    title_str = sprintf('%d点 (%.1f%%)', size(pc_refined,1), ...
                        100*size(pc_refined,1)/size(pc_cand,1));
    text(mean(pc_refined(:,1)), mean(pc_refined(:,2)), max(pc_refined(:,3)), title_str, ...
         'FontSize', 9, 'FontName', 'SimHei', 'HorizontalAlignment', 'center', ...
         'BackgroundColor', [1 1 1 0.8]);
    axis equal; grid on; view(45, 30);
end

sgtitle('图5-11-3: 基于法向量约束与空间连通性的点云精化', ...
        'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimHei');

end
