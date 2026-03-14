% fig5_11_2.m - 图5-11-2: 基于高度分层的引脚底部接触区提取
%
% 功能：通过分析引脚点云的高度分布直方图，自适应提取底部接触区候选点云
%
% 解决问题：
%   1. 引脚点云包含根部、延伸段和底部三个区段，需分离底部
%   2. 高度直方图呈现多峰分布，底部对应最低高度峰值
%   3. 需自适应确定高度阈值以容纳底部轻微倾斜
%
% 核心方法：
%   高度直方图分析：
%     将高度范围均匀划分为N_bin个区间，统计各区间点数
%   最低峰值识别：
%     在直方图低高度一侧识别峰值，定位底部主要高度区域
%   自适应阈值：
%     T_z = Z_median_low + k_z * sigma_z_low
%     k_z为阈值系数（通常1.5~2.5），根据底部平整度调整
%   候选点云提取：
%     保留高度低于阈值的所有点作为底部接触区初步候选
%
% 输入：
%   X - X坐标矩阵 (H×W)
%   Y - Y坐标矩阵 (H×W)
%   Z - Z坐标矩阵 (H×W)
%   pin_regions - 引脚区域结构体（来自fig5_10_3），包含字段：
%     .top/bottom/left/right - 各侧结构体，包含：
%       .num_pins - 引脚数量
%       .mask_pins - 该侧所有引脚的合并掩膜 (H×W logical)
%       .pins - 引脚信息结构体数组，每个引脚包含：
%         .centroid - 质心坐标 [x, y]
%         .area - 面积
%         .bbox - 边界框 [x, y, width, height]
%   params - 参数结构体（可选）：
%     .k_z - 高度阈值系数（默认2.0）
%     .n_bins - 直方图区间数（默认50）
%
% 输出：
%   pins_bottom_candidates - 引脚底部候选点云结构体数组（1×N_pins）：
%     [i].pin_id - 引脚ID
%     [i].side - 引脚所属侧（'top'/'bottom'/'left'/'right'）
%     [i].point_cloud_full - 引脚完整点云 (M×3)
%     [i].point_cloud_bottom - 底部候选点云 (K×3，K<M)
%     [i].height_hist - 高度直方图结构 (bin_centers, counts)
%     [i].z_threshold - 高度阈值
%     [i].z_median_low - 低高度区中位数
%     [i].z_sigma_low - 低高度区标准差
%
% 使用方法：
%   load('X.mat', 'X'); load('Y.mat', 'Y'); load('Z.mat', 'Z');
%   load('qfp_pin_localization_results.mat', 'pin_regions');  % 从segment结果加载
%   pins_bottom = fig5_11_2(X, Y, Z, pin_regions);
%   或运行测试脚本：test_fig5_11_2

function pins_bottom_candidates = QFP_Cal_Step2_PinTipExtraction(X, Y, Z, pin_regions, params)

fprintf('====== 图5-11-2: 基于高度分层的引脚底部接触区提取 ======\n');

%% 1. 参数初始化
if nargin < 5 || isempty(params)
    params = struct();
end
if ~isfield(params, 'k_z'), params.k_z = 2.0; end
if ~isfield(params, 'n_bins'), params.n_bins = 50; end

fprintf('参数设置:\n');
fprintf('  k_z (高度阈值系数) = %.2f\n', params.k_z);
fprintf('  n_bins (直方图区间数) = %d\n', params.n_bins);

%% 2. 收集所有引脚信息并生成mask
sides = {'top', 'bottom', 'left', 'right'};
all_pins = [];
pin_id_global = 1;

[H, W] = size(Z);

for s = 1:length(sides)
    side_name = sides{s};
    if isfield(pin_regions, side_name) && isfield(pin_regions.(side_name), 'pins')
        pins_side = pin_regions.(side_name).pins;
        
        % 获取该侧的引脚总掩膜
        if isfield(pin_regions.(side_name), 'mask_pins')
            mask_pins_side = pin_regions.(side_name).mask_pins;
        else
            mask_pins_side = [];
        end
        
        for i = 1:length(pins_side)
            pin_info = struct();
            pin_info.pin_id = pin_id_global;
            pin_info.side = side_name;
            
            % 检查是否已有mask字段
            if isfield(pins_side(i), 'mask')
                pin_info.mask = pins_side(i).mask;
            else
                % 从bbox和侧边掩膜中提取mask
                if isfield(pins_side(i), 'bbox') && ~isempty(mask_pins_side)
                    bbox = pins_side(i).bbox;
                    x = round(bbox(1));
                    y = round(bbox(2));
                    w = round(bbox(3));
                    h = round(bbox(4));
                    
                    % 确保bbox在图像范围内
                    x = max(1, min(W, x));
                    y = max(1, min(H, y));
                    x_end = min(W, x + w - 1);
                    y_end = min(H, y + h - 1);
                    
                    % 提取bbox区域的掩膜
                    mask_roi = false(H, W);
                    mask_roi(y:y_end, x:x_end) = mask_pins_side(y:y_end, x:x_end);
                    
                    % 连通域分析，选择质心最接近bbox中心的区域
                    CC = bwconncomp(mask_roi);
                    if CC.NumObjects > 0
                        bbox_center = [x + w/2, y + h/2];
                        min_dist = inf;
                        best_region = 1;
                        
                        for k = 1:CC.NumObjects
                            [rows, cols] = ind2sub([H, W], CC.PixelIdxList{k});
                            centroid_k = [mean(cols), mean(rows)];
                            dist_k = norm(centroid_k - bbox_center);
                            if dist_k < min_dist
                                min_dist = dist_k;
                                best_region = k;
                            end
                        end
                        
                        % 生成该引脚的mask
                        pin_mask = false(H, W);
                        pin_mask(CC.PixelIdxList{best_region}) = true;
                        pin_info.mask = pin_mask;
                    else
                        % 如果没有找到连通域，使用整个bbox区域
                        pin_info.mask = mask_roi;
                    end
                else
                    % 既没有mask也没有bbox，创建空mask
                    pin_info.mask = false(H, W);
                end
            end
            
            all_pins = [all_pins; pin_info];
            pin_id_global = pin_id_global + 1;
        end
    end
end

N_pins = length(all_pins);
fprintf('\n检测到 %d 个引脚\n', N_pins);

%% 3. 提取各引脚的点云并执行高度分层
pins_bottom_candidates = struct('pin_id', {}, 'side', {}, ...
    'point_cloud_full', {}, 'point_cloud_bottom', {}, ...
    'height_hist', {}, 'z_threshold', {}, 'z_lower', {}, ...
    'z_median_low', {}, 'z_sigma_low', {});

fprintf('\n开始处理各引脚...\n');
for i = 1:N_pins
    pin = all_pins(i);
    fprintf('  [%d/%d] 引脚ID=%d, 侧=%s\n', i, N_pins, pin.pin_id, pin.side);
    
    % 提取引脚点云
    X_pin = X(pin.mask);
    Y_pin = Y(pin.mask);
    Z_pin = Z(pin.mask);
    
    % 剔除无效点
    valid_idx = (Z_pin ~= 0) & ~isnan(Z_pin) & ~isnan(X_pin) & ~isnan(Y_pin);
    X_pin = X_pin(valid_idx);
    Y_pin = Y_pin(valid_idx);
    Z_pin = Z_pin(valid_idx);
    
    % 统计滤波：基于KNN距离去除杂点
    if length(Z_pin) > 30
        pc_temp = [X_pin(:), Y_pin(:), Z_pin(:)];
        [pc_temp, sf_valid] = statistical_filter_pc(pc_temp, 10, 2.0);
        n_removed = sum(~sf_valid);
        if n_removed > 0
            X_pin = pc_temp(:,1);
            Y_pin = pc_temp(:,2);
            Z_pin = pc_temp(:,3);
            fprintf('    统计滤波去除 %d 个杂点\n', n_removed);
        end
    end
    
    if length(Z_pin) < 20
        fprintf('    警告: 有效点数过少(%d)，跳过\n', length(Z_pin));
        continue;
    end
    
    pc_full = [X_pin(:), Y_pin(:), Z_pin(:)];
    fprintf('    完整点云: %d 点\n', size(pc_full, 1));
    
    % 高度直方图分析
    z_min = min(Z_pin);
    z_max = max(Z_pin);
    z_range = z_max - z_min;
    
    if z_range < 1e-6
        fprintf('    警告: 高度范围过小(%.4f)，跳过\n', z_range);
        continue;
    end
    
    % 计算直方图
    bin_edges = linspace(z_min, z_max, params.n_bins + 1);
    bin_centers = (bin_edges(1:end-1) + bin_edges(2:end)) / 2;
    [counts, ~] = histcounts(Z_pin, bin_edges);
    bin_width = bin_centers(2) - bin_centers(1);
    
    %% ---- 改进：基于直方图峰值检测定位真实底部区域 ----
    % 平滑直方图以稳健检测峰值
    counts_smooth = movmean(counts, 3);
    
    % 寻找显著峰值（最小高度为最大计数的5%）
    min_peak_height = max(counts_smooth) * 0.05;
    [peak_vals, peak_locs] = findpeaks(counts_smooth, 'MinPeakHeight', min_peak_height);
    
    if ~isempty(peak_locs)
        % 最低显著峰值 = 底部接触区的主峰
        lowest_peak_bin = peak_locs(1);
        lowest_peak_z = bin_centers(lowest_peak_bin);
        
        % 提取峰值附近的点来计算统计量
        peak_half_range = 3 * bin_width;
        peak_pts_idx = (Z_pin >= lowest_peak_z - peak_half_range) & ...
                       (Z_pin <= lowest_peak_z + peak_half_range);
        Z_peak = Z_pin(peak_pts_idx);
        
        if length(Z_peak) >= 5
            z_median_low = median(Z_peak);
            z_sigma_low = std(Z_peak);
        else
            z_median_low = lowest_peak_z;
            z_sigma_low = bin_width;
        end
        
        % 确定上界阈值
        if length(peak_locs) > 1
            % 多峰：找最低峰与次低峰之间的谷底作为上界
            next_peak_bin = peak_locs(2);
            valley_region = counts_smooth(lowest_peak_bin:next_peak_bin);
            [~, valley_offset] = min(valley_region);
            valley_idx = lowest_peak_bin + valley_offset - 1;
            z_upper = bin_centers(valley_idx);
            % 上界不超过 median + k_z * sigma
            z_upper = min(z_upper, z_median_low + params.k_z * z_sigma_low);
        else
            z_upper = z_median_low + params.k_z * z_sigma_low;
        end
        
        % 确定下界阈值（排除底部噪声点）
        z_lower = z_median_low - params.k_z * z_sigma_low;
        
    else
        % 无显著峰值，回退使用原始方法
        z_30pct = z_min + 0.3 * z_range;
        low_idx_fb = Z_pin <= z_30pct;
        if sum(low_idx_fb) >= 5
            Z_low_fb = Z_pin(low_idx_fb);
            z_median_low = median(Z_low_fb);
            z_sigma_low = std(Z_low_fb);
        else
            z_median_low = z_min + 0.15 * z_range;
            z_sigma_low = 0.1 * z_range;
        end
        z_upper = z_median_low + params.k_z * z_sigma_low;
        z_lower = z_min;  % 回退时不设下界
    end
    
    z_threshold = z_upper;
    
    % 提取底部候选点云（双向阈值）
    bottom_idx = (Z_pin >= z_lower) & (Z_pin <= z_threshold);
    pc_bottom = pc_full(bottom_idx, :);
    
    % 密度清理：去除孤立噪声点
    if size(pc_bottom, 1) > 10
        [pc_bottom, density_valid] = density_cleanup(pc_bottom, 5, 0.3);
        n_density_removed = sum(~density_valid);
        if n_density_removed > 0
            fprintf('    密度清理去除 %d 个孤立点\n', n_density_removed);
        end
    end
    
    fprintf('    高度范围: [%.4f, %.4f] mm\n', z_min, z_max);
    fprintf('    底部峰值: %.4f mm, 标准差: %.4f mm\n', z_median_low, z_sigma_low);
    fprintf('    阈值范围: [%.4f, %.4f] mm\n', z_lower, z_threshold);
    fprintf('    底部候选点数: %d (%.1f%%)\n', size(pc_bottom, 1), ...
            100*size(pc_bottom,1)/size(pc_full,1));
    
    % 保存结果
    result = struct();
    result.pin_id = pin.pin_id;
    result.side = pin.side;
    result.point_cloud_full = pc_full;
    result.point_cloud_bottom = pc_bottom;
    result.height_hist = struct('bin_centers', bin_centers, 'counts', counts);
    result.z_threshold = z_threshold;
    result.z_lower = z_lower;
    result.z_median_low = z_median_low;
    result.z_sigma_low = z_sigma_low;
    
    pins_bottom_candidates(end+1) = result;
end

fprintf('\n成功处理 %d 个引脚\n', length(pins_bottom_candidates));

%% 4. 可视化（选取若干典型引脚）
fprintf('\n生成可视化结果（按侧分组显示）...\n');
visualize_height_stratification(pins_bottom_candidates);

fprintf('====== 高度分层提取完成 ======\n\n');

end

%% ========== 辅助函数：可视化（按侧分组） ==========
function visualize_height_stratification(pins_bottom)

if isempty(pins_bottom)
    fprintf('  无有效引脚可视化\n');
    return;
end

% 按侧分组
sides = {'left', 'right', 'top', 'bottom'};
for s_idx = 1:length(sides)
    side_name = sides{s_idx};
    pins_side = pins_bottom(strcmp({pins_bottom.side}, side_name));
    
    if isempty(pins_side)
        continue;
    end
    
    n_pins = length(pins_side);
    fprintf('  可视化%s侧: %d个引脚\n', side_name, n_pins);
    
    % 创建该侧的figure
    fig = figure('Position', [100 + (s_idx-1)*100, 100, 1600, 400*n_pins], ...
                 'Color', 'w', 'Name', sprintf('图5-11-2-%s侧', side_name));
    for idx = 1:n_pins
        pin = pins_side(idx);
        pc_full = pin.point_cloud_full;
        pc_bottom = pin.point_cloud_bottom;
        hist_data = pin.height_hist;
        
        %% 子图(a): 引脚点云完整分布
        subplot(n_pins, 4, (idx-1)*4 + 1);
        scatter3(pc_full(:,1), pc_full(:,2), pc_full(:,3), 15, pc_full(:,3), 'filled');
        colormap(gca, jet);
        xlabel('X(mm)', 'FontSize', 9, 'FontName', 'SimHei');
        ylabel('Y(mm)', 'FontSize', 9, 'FontName', 'SimHei');
        zlabel('Z(mm)', 'FontSize', 9, 'FontName', 'SimHei');
        if idx == 1
            title('(a) 引脚点云完整分布', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
        end
        title_str = sprintf('引脚%d (%s侧)\n%d点', pin.pin_id, pin.side, size(pc_full,1));
        text(mean(pc_full(:,1)), mean(pc_full(:,2)), max(pc_full(:,3)), title_str, ...
             'FontSize', 9, 'FontName', 'SimHei', 'HorizontalAlignment', 'center', ...
             'BackgroundColor', [1 1 1 0.7]);
        axis equal; grid on; view(45, 30);
        
        %% 子图(b): 高度直方图与峰值识别
        subplot(n_pins, 4, (idx-1)*4 + 2);
        bar(hist_data.bin_centers, hist_data.counts, 'FaceColor', [0.4 0.6 0.9], 'EdgeColor', 'k');
        hold on;
        % 标注中位数和双向阈值
        y_max = max(hist_data.counts);
        plot([pin.z_median_low pin.z_median_low], [0 y_max], 'g--', 'LineWidth', 2);
        plot([pin.z_threshold pin.z_threshold], [0 y_max], 'r--', 'LineWidth', 2);
        if isfield(pin, 'z_lower')
            plot([pin.z_lower pin.z_lower], [0 y_max], 'm--', 'LineWidth', 2);
            legend({'直方图', '底部峰值', '上界阈值', '下界阈值'}, 'Location', 'best', 'FontSize', 8, 'FontName', 'SimHei');
        else
            legend({'直方图', '中位数', '阈值'}, 'Location', 'best', 'FontSize', 8, 'FontName', 'SimHei');
        end
        xlabel('高度 Z (mm)', 'FontSize', 9, 'FontName', 'SimHei');
        ylabel('点数', 'FontSize', 9, 'FontName', 'SimHei');
        if idx == 1
            title('(b) 高度直方图与峰值识别', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
        end
        grid on;
        
        %% 子图(c): 高度阈值设定说明
        subplot(n_pins, 4, (idx-1)*4 + 3);
        axis off;
        if isfield(pin, 'z_lower')
            z_lower_val = pin.z_lower;
        else
            z_lower_val = -inf;
        end
        
        % 创建包含中文和数值的文本，分两部分显示
        y_pos = 0.95;
        line_height = 0.06;
        
        % 中文标签部分 - 使用SimHei
        labels = {'引脚ID:', '侧:', '', '底部峰值统计:', '  峰值中心:', '  标准差:', ...
                  '', '双向阈值:', '  下界:', '  上界:', '', '结果:', '  总点数:', '  底部候选:', '  比例:'};
        % 数值部分 - 使用Consolas
        values = {sprintf('%d', pin.pin_id), pin.side, '', '', ...
                  sprintf('%.4f mm', pin.z_median_low), sprintf('%.4f mm', pin.z_sigma_low), ...
                  '', '', sprintf('%.4f mm', z_lower_val), sprintf('%.4f mm', pin.z_threshold), ...
                  '', '', sprintf('%d', size(pc_full,1)), sprintf('%d', size(pc_bottom,1)), ...
                  sprintf('%.1f%%', 100*size(pc_bottom,1)/size(pc_full,1))};
        
        for i = 1:length(labels)
            if ~isempty(labels{i})
                % 中文标签
                text(0.05, y_pos, labels{i}, 'FontSize', 9, 'FontName', 'SimHei', ...
                     'VerticalAlignment', 'top', 'Units', 'normalized', 'Interpreter', 'none');
                % 数值
                if ~isempty(values{i})
                    text(0.50, y_pos, values{i}, 'FontSize', 9, 'FontName', 'Consolas', ...
                         'VerticalAlignment', 'top', 'Units', 'normalized', 'Interpreter', 'none');
                end
            end
            y_pos = y_pos - line_height;
        end
        
        if idx == 1
            title('(c) 高度阈值设定', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
        end
        
        %% 子图(d): 底部接触区候选点云
        subplot(n_pins, 4, (idx-1)*4 + 4);
        % 绘制完整点云（灰色透明）
        scatter3(pc_full(:,1), pc_full(:,2), pc_full(:,3), 10, [0.7 0.7 0.7], ...
                 'filled', 'MarkerFaceAlpha', 0.2);
        hold on;
        % 绘制底部候选点云（彩色高亮）
        if ~isempty(pc_bottom)
            scatter3(pc_bottom(:,1), pc_bottom(:,2), pc_bottom(:,3), 25, pc_bottom(:,3), 'filled');
        end
        colormap(gca, jet);
        xlabel('X(mm)', 'FontSize', 9, 'FontName', 'SimHei');
        ylabel('Y(mm)', 'FontSize', 9, 'FontName', 'SimHei');
        zlabel('Z(mm)', 'FontSize', 9, 'FontName', 'SimHei');
        if idx == 1
            title('(d) 底部接触区候选点云', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
        end
        legend({'完整点云', '底部候选'}, 'Location', 'best', 'FontSize', 8, 'FontName', 'SimHei');
        axis equal; grid on; view(45, 30);
    end
    
    sgtitle(sprintf('图5-11-2: 基于高度分层的引脚底部接触区提取（%s侧）', side_name), ...
            'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimHei');
end

end

%% ========== 辅助函数：基于KNN距离的统计滤波 ==========
function [pc_filtered, valid_idx] = statistical_filter_pc(pc, k_nn, alpha)
% 基于K近邻平均距离的统计离群点滤波
% 输入:
%   pc    - Nx3 点云矩阵
%   k_nn  - 近邻数 (默认10)
%   alpha - 阈值系数 (默认2.0)，mean + alpha*std
% 输出:
%   pc_filtered - 滤波后的点云
%   valid_idx   - 保留点的逻辑索引

if nargin < 2, k_nn = 10; end
if nargin < 3, alpha = 2.0; end

N = size(pc, 1);
if N <= k_nn + 1
    pc_filtered = pc;
    valid_idx = true(N, 1);
    return;
end

% 计算每个点到K近邻的平均距离
mean_dists = zeros(N, 1);
for j = 1:N
    diffs = pc - pc(j, :);
    dists = sqrt(sum(diffs.^2, 2));
    dists_sorted = sort(dists);
    mean_dists(j) = mean(dists_sorted(2:min(k_nn+1, N)));
end

% 全局统计
mu_d = mean(mean_dists);
sigma_d = std(mean_dists);
threshold = mu_d + alpha * sigma_d;

% 滤波
valid_idx = mean_dists <= threshold;
pc_filtered = pc(valid_idx, :);

end

%% 密度清理：去除孤立噪声点
% pc: Nx3 点云
% k_nn: 近邻数量
% quantile_ratio: 最大平均近邻距离的分位数阈值（0~1），
%   超过该分位数对应距离的点被认为是孤立噪声
function [pc_clean, valid_idx] = density_cleanup(pc, k_nn, quantile_ratio)
    N = size(pc, 1);
    if N <= k_nn
        pc_clean = pc;
        valid_idx = true(N, 1);
        return;
    end
    
    % 计算每个点的k近邻平均距离
    mean_dists = zeros(N, 1);
    for j = 1:N
        diffs = pc - pc(j, :);
        dists = sqrt(sum(diffs.^2, 2));
        dists_sorted = sort(dists);
        mean_dists(j) = mean(dists_sorted(2:min(k_nn+1, N)));
    end
    
    % 使用分位数作为阈值：距离大于q分位数的点视为孤立
    dist_threshold = quantile(mean_dists, 1 - quantile_ratio);
    valid_idx = mean_dists <= dist_threshold;
    pc_clean = pc(valid_idx, :);
end
