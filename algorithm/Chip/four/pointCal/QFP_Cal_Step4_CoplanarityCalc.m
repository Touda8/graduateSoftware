% fig5_11_4.m - 引脚平面拟合、JEDEC座落平面共面度与翘曲度计算
%
% ===== 核心思想 =====
%
% 不使用任何"代表点"（质心、最低点等）来衡量共面度。
% 直接用每个引脚的底部点云与座落平面的几何关系来计算。
%
% 物理模型：将芯片自然放置在无限大刚性平面（PCB）上，
% 重力使芯片稳定在3个非共线引脚的支撑上。
% 此支撑面 = JEDEC座落平面（切平面）。
%
% ===== 算法流程 =====
%
% 步骤1：逐引脚PCA平面拟合 + Tukey迭代剔除离群点
%   - 获取每个引脚的干净底部点云和局部平面参数
%
% 步骤2：确定全局"底部"方向（不假定Z轴朝向）
%   - 合并所有引脚底部点云 → PCA → 最小特征值方向 = 法向量
%   - 通过投影分布确定哪侧为底部
%
% 步骤3：JEDEC座落平面（切平面）拟合
%   - 对每个引脚，沿全局法向量方向找到其点云中最底部的点（floor point）
%   - 按floor point的投影值排序所有引脚
%   - 3个最底部且非共线的引脚的floor point定义座落平面
%   - 座落平面 = 从下方托住芯片的支撑面
%
% 步骤4：共面度计算（直接从点云，无需代表点）
%   - 对每个引脚：计算其所有点到座落平面的有符号距离
%   - 该引脚的共面度偏差 = min(有符号距离) = 其最靠近PCB的点到座落面的间隙
%   - 物理意义：引脚底面与PCB之间的最小间距
%   - JEDEC共面度 = max(各引脚偏差) = 最大间隙
%   - 座落引脚偏差≈0（接触PCB），翘曲引脚偏差>0（悬空）
%
% 步骤5：逐引脚翘曲度
%   - 翘曲角 = 引脚局部平面法向量与座落平面法向量的夹角
%   - 翘曲高度 = 引脚点云沿座落法向的投影跨度
%
% ===== 输入 =====
%   pins_bottom_refined - 来自fig5_11_3的精化点云结构体数组
%     [i].pin_id                     - 引脚ID
%     [i].side                       - 引脚侧别
%     [i].point_cloud_bottom_refined - 精化后底部点云 N×3
%   params - 参数结构体（可选）：
%     .max_iter  - Tukey迭代最大次数（默认5）
%     .kappa     - Tukey IQR系数（默认1.5）
%
% ===== 输出 =====
%   coplanarity_result - 结构体：
%     .pins(i)              - 各引脚详情
%       .pin_id             - 引脚ID
%       .side               - 侧别
%       .plane_params       - 局部平面 [A B C D]
%       .plane_normal       - 局部法向量
%       .rmse               - 拟合RMSE
%       .n_points           - 有效点数
%       .point_cloud        - 清理后点云
%       .iterations         - 迭代次数
%       .deviation          - 到座落平面的最近距离（共面度偏差）
%       .warp_angle         - 翘曲角(°)
%       .warp_height        - 翘曲高度(mm)
%     .seating_plane        - 座落平面 {.normal, .d, .points}
%     .seating_pin_ids      - 3个座落引脚ID
%     .seating_pin_indices  - 3个座落引脚索引
%     .reference_plane      - Z=AX+BY+C格式 [A B C]
%     .coplanarity_value    - JEDEC共面度(mm)
%     .max_deviation / .min_deviation / .deviation_std
%     .deviations           - 所有引脚偏差向量
%     .warp_angles / .warp_heights
%     .statistics_by_side   - 各侧统计

function coplanarity_result = QFP_Cal_Step4_CoplanarityCalc(pins_bottom_refined, params)

fprintf('====== 引脚平面拟合、座落平面共面度与翘曲度计算 ======\n');

%% 1. 参数初始化
if nargin < 2 || isempty(params)
    params = struct();
end
if ~isfield(params, 'max_iter'), params.max_iter = 5; end
if ~isfield(params, 'kappa'), params.kappa = 1.5; end

fprintf('参数: max_iter=%d, kappa=%.2f\n', params.max_iter, params.kappa);

%% ============================================================
%%  步骤1: 逐引脚PCA平面拟合 + Tukey离群点剔除
%% ============================================================
N_pins = length(pins_bottom_refined);
fprintf('\n步骤1: 逐引脚PCA平面拟合 (%d个引脚)...\n', N_pins);

% 预定义所有字段，避免struct动态增长字段不一致导致赋值报错
pins_fitted = struct( ...
    'pin_id',       cell(1,0), ...
    'side',         cell(1,0), ...
    'plane_params', cell(1,0), ...
    'plane_normal', cell(1,0), ...
    'rmse',         cell(1,0), ...
    'n_points',     cell(1,0), ...
    'point_cloud',  cell(1,0), ...
    'iterations',   cell(1,0), ...
    'deviation',    cell(1,0), ...
    'warp_angle',   cell(1,0), ...
    'warp_height',  cell(1,0));

for i = 1:N_pins
    pin = pins_bottom_refined(i);
    fprintf('  [%d/%d] 引脚%d (%s)\n', i, N_pins, pin.pin_id, pin.side);
    
    pc = pin.point_cloud_bottom_refined;
    N_pts = size(pc, 1);
    
    if N_pts < 5
        fprintf('    跳过: 点数过少(%d)\n', N_pts);
        continue;
    end
    
    % --- PCA + Tukey迭代 ---
    pc_cur = pc;
    iter = 0;
    converged = false;
    
    while iter < params.max_iter && ~converged
        iter = iter + 1;
        N_cur = size(pc_cur, 1);
        
        mu = mean(pc_cur, 1);
        C_cov = ((pc_cur - mu)' * (pc_cur - mu)) / N_cur;
        [V, D] = eig(C_cov);
        [~, si] = sort(diag(D), 'ascend');
        n_vec = V(:, si(1))';   % 最小特征值方向 = 法向量
        
        d_plane = -dot(n_vec, mu);
        dists = abs(pc_cur * n_vec' + d_plane);
        rmse = sqrt(mean(dists.^2));
        
        if N_cur >= 10
            Q1 = quantile(dists, 0.25);
            Q3 = quantile(dists, 0.75);
            T = Q3 + params.kappa * (Q3 - Q1);
            keep = dists <= T;
            n_rm = sum(~keep);
            pc_cur = pc_cur(keep, :);
            
            fprintf('    迭代%d: RMSE=%.4fmm, 剔除%d, 剩余%d\n', ...
                    iter, rmse, n_rm, size(pc_cur, 1));
            if n_rm == 0 || size(pc_cur, 1) < 5
                converged = true;
            end
        else
            converged = true;
        end
    end
    
    % --- 最终平面拟合（剔除后重新拟合确保一致性）---
    N_final = size(pc_cur, 1);
    mu_f = mean(pc_cur, 1);
    C_f = ((pc_cur - mu_f)' * (pc_cur - mu_f)) / N_final;
    [V_f, D_f] = eig(C_f);
    [~, si_f] = sort(diag(D_f), 'ascend');
    n_final = V_f(:, si_f(1))';
    
    % 法向量方向暂不固定（步骤2统一确定底部方向）
    d_final = -dot(n_final, mu_f);
    pp = [n_final, d_final];  % [A B C D]
    
    dists_final = abs(pc_cur * n_final' + d_final);
    rmse_final = sqrt(mean(dists_final.^2));
    
    fprintf('    最终: %d点, RMSE=%.4fmm\n', N_final, rmse_final);
    
    % --- 保存（deviation/warp由后续步骤填充）---
    r = struct();
    r.pin_id      = pin.pin_id;
    r.side        = pin.side;
    r.plane_params = pp;
    r.plane_normal = n_final;
    r.rmse        = rmse_final;
    r.n_points    = N_final;
    r.point_cloud = pc_cur;
    r.iterations  = iter;
    r.deviation   = 0;     % 步骤4填充
    r.warp_angle  = 0;     % 步骤5填充
    r.warp_height = 0;     % 步骤5填充
    
    pins_fitted(end+1) = r; %#ok<AGROW>
end

N_valid = length(pins_fitted);
fprintf('成功拟合 %d 个引脚\n', N_valid);
if N_valid < 3
    error('有效引脚不足3个，无法计算共面度');
end

%% ============================================================
%%  步骤2: 确定全局"底部"方向（不假定Z轴朝向）
%% ============================================================
fprintf('\n步骤2: 确定全局底部方向...\n');

% 合并所有引脚的清理后底部点云
all_pts = [];
for i = 1:N_valid
    all_pts = [all_pts; pins_fitted(i).point_cloud]; %#ok<AGROW>
end

% PCA获取全局法向量（所有引脚底面共同的法向方向）
mu_all = mean(all_pts, 1);
centered_all = all_pts - mu_all;
C_all = (centered_all' * centered_all) / size(all_pts, 1);
[V_all, D_all] = eig(C_all);
[~, sort_all] = sort(diag(D_all), 'ascend');
global_normal = V_all(:, sort_all(1))';  % 最小特征值 = 法向量

% 统一各引脚局部法向量的朝向（与全局法向量一致）
for i = 1:N_valid
    if dot(pins_fitted(i).plane_normal, global_normal) < 0
        pins_fitted(i).plane_normal = -pins_fitted(i).plane_normal;
        pp = pins_fitted(i).plane_params;
        pins_fitted(i).plane_params = [-pp(1:3), -pp(4)];
    end
end

fprintf('  全局法向量: [%.4f, %.4f, %.4f]\n', global_normal);

%% ============================================================
%%  步骤3: JEDEC座落平面（切平面）拟合
%% ============================================================
fprintf('\n步骤3: JEDEC座落平面拟合...\n');

% 对每个引脚，沿全局法向量找到其点云中最底部的点（floor point）
% floor point = 投影值最小的点 = 离PCB最近的点
floor_projs  = zeros(N_valid, 1);   % 各引脚floor point的投影值
floor_points = zeros(N_valid, 3);   % 各引脚floor point的3D坐标

for i = 1:N_valid
    pc_i = pins_fitted(i).point_cloud;
    proj_i = pc_i * global_normal';   % 投影到全局法向量
    [floor_projs(i), min_idx] = min(proj_i);
    floor_points(i, :) = pc_i(min_idx, :);
end

% 确定法向量朝向：让floor point在"下方"，法向量指向"上方"
% 约定：法向量指向远离PCB的方向（pointing up/away from seating plane）
% 此时 floor_projs 应为较小的值（底部），芯片本体在较大值的方向
% 检查：如果floor_projs的离散度合理，说明方向正确
% 反之取反：简单策略 - 让法向量指向使全部投影为正的方向
median_proj = median(all_pts * global_normal');
if median_proj < mean(floor_projs)
    % 大多数点都在floor point下方→方向反了
    global_normal = -global_normal;
    floor_projs = -floor_projs;
    floor_points_tmp = zeros(N_valid, 3);
    for i = 1:N_valid
        pc_i = pins_fitted(i).point_cloud;
        proj_i = pc_i * global_normal';
        [floor_projs(i), min_idx] = min(proj_i);
        floor_points_tmp(i, :) = pc_i(min_idx, :);
    end
    floor_points = floor_points_tmp;
    % 同步翻转局部法向量
    for i = 1:N_valid
        pins_fitted(i).plane_normal = -pins_fitted(i).plane_normal;
        pp = pins_fitted(i).plane_params;
        pins_fitted(i).plane_params = [-pp(1:3), -pp(4)];
    end
end

fprintf('  底部方向确定完毕, 法向量: [%.4f, %.4f, %.4f]\n', global_normal);

% 按floor投影值升序排列（最低的在前）
[~, proj_order] = sort(floor_projs, 'ascend');

% 搜索3个最低且非共线的引脚 → 定义座落平面
xyz_span = max(max(floor_points) - min(floor_points));
min_area_thr = (0.01 * xyz_span)^2;   % 非共线性阈值

found_seating = false;
seat_idx = [0 0 0];
for ii = 1:N_valid-2
    for jj = ii+1:N_valid-1
        for kk = jj+1:N_valid
            idx3 = proj_order([ii, jj, kk]);
            p1 = floor_points(idx3(1), :);
            p2 = floor_points(idx3(2), :);
            p3 = floor_points(idx3(3), :);
            tri_area = 0.5 * norm(cross(p2 - p1, p3 - p1));
            if tri_area > min_area_thr
                found_seating = true;
                seat_idx = idx3(:)';
                break;
            end
        end
        if found_seating, break; end
    end
    if found_seating, break; end
end

if ~found_seating
    warning('找不到3个非共线座落引脚，使用3个最低引脚');
    seat_idx = proj_order(1:3)';
end

% 由3个floor point确定座落平面
sp1 = floor_points(seat_idx(1), :);
sp2 = floor_points(seat_idx(2), :);
sp3 = floor_points(seat_idx(3), :);

seat_normal = cross(sp2 - sp1, sp3 - sp1);
seat_normal = seat_normal / norm(seat_normal);

% 保证座落法向量指向"上方"（与全局法向量同向）
if dot(seat_normal, global_normal) < 0
    seat_normal = -seat_normal;
end

seat_d = -dot(seat_normal, sp1);   % 平面方程: n·p + d = 0

seating_plane = struct('normal', seat_normal, 'd', seat_d, ...
                       'points', [sp1; sp2; sp3]);

seat_ids = [pins_fitted(seat_idx(1)).pin_id, ...
            pins_fitted(seat_idx(2)).pin_id, ...
            pins_fitted(seat_idx(3)).pin_id];

fprintf('  座落引脚ID: [%d, %d, %d]\n', seat_ids);
fprintf('  座落法向量: [%.4f, %.4f, %.4f]\n', seat_normal);

%% ============================================================
%%  步骤4: 共面度计算（直接从点云，不使用代表点）
%% ============================================================
fprintf('\n步骤4: 共面度计算（点云直接法）...\n');

deviations = zeros(N_valid, 1);

for i = 1:N_valid
    pc_i = pins_fitted(i).point_cloud;
    
    % 计算该引脚所有点到座落平面的有符号距离
    % 正值 = 在座落平面上方（悬空侧）
    % ≈0  = 在座落平面上（接触PCB）
    signed_dists = pc_i * seat_normal' + seat_d;
    
    % 该引脚的共面度偏差 = 其点云中最靠近座落平面的点的距离
    % = min(signed_dists)
    % 物理意义：如果PCB在座落平面位置，该引脚底面与PCB之间的最小间隙
    deviations(i) = min(signed_dists);
    
    pins_fitted(i).deviation = deviations(i);
end

% JEDEC共面度 = 最大间隙（离PCB最远的引脚）
coplanarity_value = max(deviations);

fprintf('  JEDEC共面度: %.4f mm\n', coplanarity_value);
fprintf('  偏差范围: [%.4f, %.4f] mm\n', min(deviations), max(deviations));
fprintf('  偏差标准差: %.4f mm\n', std(deviations));

%% ============================================================
%%  步骤5: 逐引脚翘曲度计算
%% ============================================================
fprintf('\n步骤5: 逐引脚翘曲度...\n');

warp_angles  = zeros(N_valid, 1);
warp_heights = zeros(N_valid, 1);

for i = 1:N_valid
    pc_i = pins_fitted(i).point_cloud;
    n_pin = pins_fitted(i).plane_normal;
    
    % 翘曲角：引脚局部平面法向量与座落平面法向量的夹角
    % 越大说明引脚弯曲越严重
    cos_a = abs(dot(n_pin, seat_normal));
    cos_a = min(cos_a, 1.0);  % 数值保护
    warp_angles(i) = acosd(cos_a);
    
    % 翘曲高度：引脚点云沿座落法向方向的最大投影跨度
    % 越大说明引脚底面越不平坦
    proj_seat = pc_i * seat_normal';
    warp_heights(i) = max(proj_seat) - min(proj_seat);
    
    pins_fitted(i).warp_angle  = warp_angles(i);
    pins_fitted(i).warp_height = warp_heights(i);
    
    fprintf('  引脚%d(%s): 偏差=%.4fmm, 翘曲角=%.2f°, 翘曲高=%.4fmm\n', ...
            pins_fitted(i).pin_id, pins_fitted(i).side, ...
            deviations(i), warp_angles(i), warp_heights(i));
end

%% ============================================================
%%  结果汇总
%% ============================================================
fprintf('\n============================================\n');
fprintf('共面度与翘曲度测量结果\n');
fprintf('============================================\n');
fprintf('座落引脚ID: [%d, %d, %d]\n', seat_ids);
fprintf('JEDEC共面度: %.4f mm\n', coplanarity_value);
fprintf('偏差均值: %.4f mm, 标准差: %.4f mm\n', mean(deviations), std(deviations));
[max_warp, max_warp_idx] = max(warp_angles);
fprintf('最大翘曲角: %.2f° (引脚%d)\n', max_warp, pins_fitted(max_warp_idx).pin_id);
fprintf('平均翘曲角: %.2f°\n', mean(warp_angles));
fprintf('============================================\n');

%% 各侧统计
fprintf('\n各侧统计:\n');
sides = {'top', 'bottom', 'left', 'right'};
stats_by_side = struct();
for s = 1:length(sides)
    sn = sides{s};
    mask = strcmp({pins_fitted.side}, sn);
    if sum(mask) > 0
        d_s = deviations(mask);
        w_s = warp_angles(mask);
        stats_by_side.(sn) = struct( ...
            'count', sum(mask), ...
            'mean_dev', mean(d_s), 'std_dev', std(d_s), ...
            'max_dev', max(d_s), 'min_dev', min(d_s), ...
            'mean_warp', mean(w_s), 'max_warp', max(w_s));
        fprintf('  %s: %d个, 偏差=%.4f±%.4fmm, 翘曲=%.2f±%.2f°\n', ...
                sn, sum(mask), mean(d_s), std(d_s), mean(w_s), std(w_s));
    else
        stats_by_side.(sn) = [];
    end
end

%% 组装输出
% 座落平面转Z=AX+BY+C格式用于可视化
if abs(seat_normal(3)) > 1e-8
    A_vis = -seat_normal(1)/seat_normal(3);
    B_vis = -seat_normal(2)/seat_normal(3);
    C_vis = -seat_d/seat_normal(3);
else
    A_vis = 0; B_vis = 0; C_vis = mean([pins_fitted.deviation]);
end

coplanarity_result = struct();
coplanarity_result.pins               = pins_fitted;
coplanarity_result.seating_plane      = seating_plane;
coplanarity_result.seating_pin_ids    = seat_ids;
coplanarity_result.seating_pin_indices = seat_idx;
coplanarity_result.reference_plane    = [A_vis, B_vis, C_vis];
coplanarity_result.coplanarity_value  = coplanarity_value;
coplanarity_result.max_deviation      = max(deviations);
coplanarity_result.min_deviation      = min(deviations);
coplanarity_result.deviation_std      = std(deviations);
coplanarity_result.deviations         = deviations;
coplanarity_result.warp_angles        = warp_angles;
coplanarity_result.warp_heights       = warp_heights;
coplanarity_result.statistics_by_side = stats_by_side;

%% 可视化
fprintf('\n生成可视化...\n');
visualize_plane_fitting(pins_fitted, min(4, N_valid));
visualize_coplanarity(coplanarity_result);
visualize_pin_clouds_seating(coplanarity_result);
visualize_warp_detail(coplanarity_result);

fprintf('\n====== 计算完成 ======\n\n');
end

%% ========== 图5-11-4: 逐引脚平面拟合可视化 ==========
function visualize_plane_fitting(pins_fitted, n_show)

n_show = min(n_show, length(pins_fitted));
figure('Position', [100 100 1600 900], 'Color', 'w', 'Name', '图5-11-4');

for idx = 1:n_show
    p = pins_fitted(idx);
    pc = p.point_cloud;
    pp = p.plane_params;   % [A B C D]
    nn = p.plane_normal;
    mu = mean(pc, 1);
    
    % 拟合平面网格
    xr = [min(pc(:,1)) max(pc(:,1))];
    yr = [min(pc(:,2)) max(pc(:,2))];
    [Xg, Yg] = meshgrid(linspace(xr(1), xr(2), 20), linspace(yr(1), yr(2), 20));
    if abs(pp(3)) > 1e-10
        Zg = -(pp(1)*Xg + pp(2)*Yg + pp(4)) / pp(3);
    else
        Zg = zeros(size(Xg));
    end
    
    % (a) 点云 + 拟合平面
    subplot(n_show, 4, (idx-1)*4+1);
    scatter3(pc(:,1), pc(:,2), pc(:,3), 15, pc(:,3), 'filled');
    hold on;
    surf(Xg, Yg, Zg, 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'FaceColor', [0.3 0.8 0.3]);
    colormap(gca, jet);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    if idx == 1
        title('(a) 点云与拟合平面', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
    end
    legend({'点云','拟合平面'}, 'FontSize', 7, 'FontName', 'SimHei', 'Location', 'best');
    axis equal; grid on; view(45,30);
    
    % (b) 残差
    subplot(n_show, 4, (idx-1)*4+2);
    res = abs(pc * nn' + pp(4));
    scatter3(pc(:,1), pc(:,2), pc(:,3), 20, res, 'filled');
    colormap(gca, jet); colorbar;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    if idx == 1
        title('(b) 残差分布', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
    end
    text(mu(1), mu(2), max(pc(:,3)), sprintf('RMSE=%.4f', p.rmse), ...
         'FontSize', 8, 'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.7]);
    axis equal; grid on; view(45,30);
    
    % (c) 残差直方图
    subplot(n_show, 4, (idx-1)*4+3);
    histogram(res, 20, 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'k');
    xlabel('残差(mm)', 'FontName', 'SimHei'); ylabel('点数', 'FontName', 'SimHei');
    if idx == 1
        title('(c) 残差直方图', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
    end
    text(0.5, 0.95, sprintf('引脚%d(%s)\n%d迭代,%d点', ...
         p.pin_id, p.side, p.iterations, p.n_points), ...
         'Units', 'normalized', 'FontSize', 8, 'FontName', 'SimHei', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
         'BackgroundColor', [1 1 0.8]);
    grid on;
    
    % (d) 平面+法向量+翘曲信息
    subplot(n_show, 4, (idx-1)*4+4);
    surf(Xg, Yg, Zg, 'FaceAlpha', 0.3, 'EdgeColor', [0.6 0.6 0.6], ...
         'FaceColor', [0.8 0.9 1.0]);
    hold on;
    scatter3(pc(:,1), pc(:,2), pc(:,3), 15, 'b', 'filled', 'MarkerFaceAlpha', 0.5);
    alen = max(xr(2)-xr(1), yr(2)-yr(1)) * 0.4;
    quiver3(mu(1), mu(2), mu(3), nn(1)*alen, nn(2)*alen, nn(3)*alen, ...
            'm', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    if idx == 1
        title('(d) 法向量与翘曲', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
    end
    text(0.02, 0.02, sprintf('翘曲角=%.2f°\n翘曲高=%.4fmm\n偏差=%.4fmm', ...
         p.warp_angle, p.warp_height, p.deviation), ...
         'Units', 'normalized', 'FontSize', 8, 'FontName', 'SimHei', ...
         'VerticalAlignment', 'bottom', 'BackgroundColor', [1 1 0.8]);
    legend({'平面','点云','法向量'}, 'FontSize', 7, 'FontName', 'SimHei', 'Location', 'best');
    axis equal; grid on; view(45,30);
end

sgtitle('图5-11-4: 引脚底面平面拟合与离群点剔除', ...
        'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
end

%% ========== 图5-11-5: 共面度与翘曲度可视化 ==========
function visualize_coplanarity(cr)

figure('Position', [100 100 1800 900], 'Color', 'w', 'Name', '图5-11-5');

pins = cr.pins;
N = length(pins);
ref = cr.reference_plane;
seat_n = cr.seating_plane.normal;

% 提取数据
pts3d = zeros(N, 3);  % 各引脚floor point（点云中离座落面最近的点）
devs  = cr.deviations;
warps = cr.warp_angles;
for i = 1:N
    pc_i = pins(i).point_cloud;
    sd = pc_i * seat_n' + cr.seating_plane.d;
    [~, mi] = min(sd);
    pts3d(i, :) = pc_i(mi, :);  % 实际floor point坐标
end

%% (a) 三维: 引脚点云 + 座落平面, 突出显示floor points
subplot(2,3,1);
% 座落平面网格
xr = [min(pts3d(:,1)) max(pts3d(:,1))];
yr = [min(pts3d(:,2)) max(pts3d(:,2))];
xm = (xr(2)-xr(1))*0.2; ym = (yr(2)-yr(1))*0.2;
xr = xr + [-xm xm]; yr = yr + [-ym ym];
[Xg, Yg] = meshgrid(linspace(xr(1),xr(2),30), linspace(yr(1),yr(2),30));
Zg = ref(1)*Xg + ref(2)*Yg + ref(3);
surf(Xg, Yg, Zg, 'FaceAlpha', 0.25, 'EdgeColor', [0.7 0.7 0.7], ...
     'FaceColor', [0.85 0.85 1.0]);
hold on;

% 各引脚floor point（按偏差着色）
scatter3(pts3d(:,1), pts3d(:,2), pts3d(:,3), 80, devs, 'filled', ...
         'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
colormap(gca, jet);
cb = colorbar; ylabel(cb, '偏差(mm)', 'FontName', 'SimHei');
if max(devs) > min(devs)
    caxis([min(devs) max(devs)]);
end

% 连线到座落面
for i = 1:N
    z_ref = ref(1)*pts3d(i,1) + ref(2)*pts3d(i,2) + ref(3);
    plot3([pts3d(i,1) pts3d(i,1)], [pts3d(i,2) pts3d(i,2)], ...
          [pts3d(i,3) z_ref], 'k--', 'LineWidth', 0.8);
end

% 标注引脚ID
for i = 1:N
    text(pts3d(i,1), pts3d(i,2), pts3d(i,3), ...
         sprintf(' %d', pins(i).pin_id), 'FontSize', 7, 'FontWeight', 'bold');
end

xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)');
title(sprintf('(a) 座落平面与引脚分布\n%d个引脚', N), ...
      'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on; view(45,30);
legend({'座落平面','引脚最近点'}, ...
       'FontSize', 8, 'FontName', 'SimHei', 'Location', 'best');

%% (b) 俯视: 共面度偏差
subplot(2,3,2);
scatter(pts3d(:,1), pts3d(:,2), 150, devs, 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
colormap(gca, jet);
cb = colorbar; ylabel(cb, '偏差(mm)', 'FontName', 'SimHei');
if max(devs) > min(devs)
    caxis([min(devs) max(devs)]);
end
hold on;
for i = 1:N
    text(pts3d(i,1), pts3d(i,2), sprintf('%d', pins(i).pin_id), ...
         'FontSize', 7, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
         'Color', 'w', 'BackgroundColor', [0 0 0 0.5]);
end
xlabel('X(mm)'); ylabel('Y(mm)');
title('(b) 共面度偏差', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on;

%% (c) 俯视: 翘曲角
subplot(2,3,3);
scatter(pts3d(:,1), pts3d(:,2), 150, warps, 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
colormap(gca, hot);
cb = colorbar; ylabel(cb, '翘曲角(°)', 'FontName', 'SimHei');
hold on;
for i = 1:N
    text(pts3d(i,1), pts3d(i,2), sprintf('%.1f°', warps(i)), ...
         'FontSize', 6, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'top', 'Color', [0 0 0.8]);
end
xlabel('X(mm)'); ylabel('Y(mm)');
title('(c) 翘曲角度', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on;

%% (d) 偏差直方图
subplot(2,3,4);
histogram(devs, 20, 'FaceColor', [0.3 0.7 0.9], 'EdgeColor', 'k');
hold on;
yl = ylim;
plot([0 0], yl, 'g-', 'LineWidth', 2);
plot([cr.coplanarity_value cr.coplanarity_value], yl, 'r--', 'LineWidth', 2);
xlabel('偏差(mm)', 'FontName', 'SimHei');
ylabel('引脚数', 'FontName', 'SimHei');
title(sprintf('(d) 偏差分布\n共面度=%.4fmm', cr.coplanarity_value), ...
      'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend({'分布','座落面','共面度'}, 'FontSize', 8, 'FontName', 'SimHei');
grid on;

text(0.98, 0.95, sprintf('JEDEC共面度: %.4f mm\n偏差标准差: %.4f mm', ...
     cr.coplanarity_value, cr.deviation_std), ...
     'Units', 'normalized', 'FontSize', 9, 'FontName', 'Consolas', ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
     'BackgroundColor', [1 1 0.9], 'EdgeColor', 'k');

%% (e) 翘曲角直方图
subplot(2,3,5);
histogram(warps, 15, 'FaceColor', [0.9 0.5 0.2], 'EdgeColor', 'k');
xlabel('翘曲角(°)', 'FontName', 'SimHei');
ylabel('引脚数', 'FontName', 'SimHei');
title(sprintf('(e) 翘曲角分布\n均值=%.2f°', mean(warps)), ...
      'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;

%% (f) 四侧箱线图
subplot(2,3,6);
sides_en = {'top','bottom','left','right'};
sides_cn = {'上侧','下侧','左侧','右侧'};
clrs = [1 .2 .2; .2 .8 .2; .8 .2 .8; .2 .8 .8];
all_d = []; all_g = {};
for s = 1:4
    m = strcmp({pins.side}, sides_en{s});
    ds = devs(m);
    if ~isempty(ds)
        all_d = [all_d; ds(:)]; %#ok<AGROW>
        all_g = [all_g; repmat(sides_cn(s), length(ds), 1)]; %#ok<AGROW>
    end
end
if ~isempty(all_d)
    boxplot(all_d, all_g, 'Colors', 'k', 'Widths', 0.5, 'Symbol', 'r+');
    hold on;
    pi2 = 0;
    for s = 1:4
        m = strcmp({pins.side}, sides_en{s});
        ds = devs(m);
        if ~isempty(ds)
            pi2 = pi2+1;
            xj = pi2 + (rand(length(ds),1)-0.5)*0.2;
            scatter(xj, ds, 40, clrs(s,:), 'filled', ...
                    'MarkerFaceAlpha', 0.6, 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
        end
    end
end
ylabel('偏差(mm)', 'FontName', 'SimHei');
title('(f) 各侧偏差对比', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;

sgtitle(sprintf('图5-11-5: QFP共面度与翘曲度 | JEDEC共面度=%.4fmm', ...
                cr.coplanarity_value), ...
        'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
end

%% ========== 图5-11-6: 所有引脚点云与座落平面（切平面）位置关系 ==========
function visualize_pin_clouds_seating(cr)

figure('Position', [50 50 1400 800], 'Color', 'w', 'Name', '图5-11-6');

pins = cr.pins;
N = length(pins);
seat_n = cr.seating_plane.normal;
seat_d = cr.seating_plane.d;
ref = cr.reference_plane;

% 收集所有点的范围
all_x = []; all_y = []; all_z = [];
for i = 1:N
    pc = pins(i).point_cloud;
    all_x = [all_x; pc(:,1)]; %#ok<AGROW>
    all_y = [all_y; pc(:,2)]; %#ok<AGROW>
    all_z = [all_z; pc(:,3)]; %#ok<AGROW>
end

% 颜色映射（按引脚偏差着色）
devs = cr.deviations;
if max(devs) > min(devs)
    dev_norm = (devs - min(devs)) / (max(devs) - min(devs));
else
    dev_norm = zeros(N, 1);
end
cmap = jet(256);

%% 左图: 三维视角
subplot(1,2,1);

% 座落平面
xr = [min(all_x) max(all_x)];
yr = [min(all_y) max(all_y)];
xm = (xr(2)-xr(1))*0.15; ym = (yr(2)-yr(1))*0.15;
xr = xr + [-xm xm]; yr = yr + [-ym ym];
[Xg, Yg] = meshgrid(linspace(xr(1),xr(2),40), linspace(yr(1),yr(2),40));
Zg = ref(1)*Xg + ref(2)*Yg + ref(3);
h_seat = surf(Xg, Yg, Zg, 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
     'FaceColor', [0.6 0.6 1.0]);
hold on;

% 绘制每个引脚的点云（不同颜色区分偏差大小）
legend_entries = {};
h_pins = [];
for i = 1:N
    pc = pins(i).point_cloud;
    ci = max(1, min(256, round(dev_norm(i)*255)+1));
    clr = cmap(ci, :);
    h = scatter3(pc(:,1), pc(:,2), pc(:,3), 8, ...
                 'MarkerFaceColor', clr, 'MarkerEdgeColor', 'none', ...
                 'MarkerFaceAlpha', 0.6);
    if i <= 3 || i == N  % 只添加少量legend避免过多
        h_pins(end+1) = h; %#ok<AGROW>
        legend_entries{end+1} = sprintf('引脚%d (%.3fmm)', pins(i).pin_id, devs(i)); %#ok<AGROW>
    end
    
    % 标注引脚ID
    mu_i = mean(pc, 1);
    text(mu_i(1), mu_i(2), mu_i(3), sprintf('%d', pins(i).pin_id), ...
         'FontSize', 7, 'FontWeight', 'bold', 'Color', clr*0.6, ...
         'HorizontalAlignment', 'center');
end

% 标记座落平面的三个定义点
sp = cr.seating_plane.points;
plot3(sp(:,1), sp(:,2), sp(:,3), 'r^', 'MarkerSize', 12, ...
      'MarkerFaceColor', 'r', 'LineWidth', 1.5);

% 座落平面法向量
mu_seat = mean(sp, 1);
arrow_len = max(xr(2)-xr(1), yr(2)-yr(1)) * 0.15;
quiver3(mu_seat(1), mu_seat(2), mu_seat(3), ...
        seat_n(1)*arrow_len, seat_n(2)*arrow_len, seat_n(3)*arrow_len, ...
        'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);

xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)');
title(sprintf('(a) 三维视角: %d个引脚点云与座落平面', N), ...
      'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend([h_seat, h_pins], ['座落平面', legend_entries], ...
       'FontSize', 7, 'FontName', 'SimHei', 'Location', 'best');
axis equal; grid on; view(45, 30);

% colorbar显示偏差映射
cb = colorbar('Position', [0.47 0.15 0.015 0.7]);
colormap(gca, jet);
if max(devs) > min(devs)
    caxis([min(devs) max(devs)]);
end
ylabel(cb, '偏差(mm)', 'FontName', 'SimHei');

%% 右图: 侧视图（沿座落平面看，突出高度差异）
subplot(1,2,2);

% 计算每个引脚点云到座落平面的有符号距离分布
for i = 1:N
    pc = pins(i).point_cloud;
    sd = pc * seat_n' + seat_d;  % 有符号距离
    ci = max(1, min(256, round(dev_norm(i)*255)+1));
    clr = cmap(ci, :);
    
    % 用引脚序号作X，有符号距离作Y → 展示各引脚点云离座落面的分布
    x_jitter = i + (rand(size(pc,1),1)-0.5)*0.5;
    scatter(x_jitter, sd, 5, clr, 'filled', 'MarkerFaceAlpha', 0.4);
    hold on;
end

% 座落平面基准线
plot([0.5 N+0.5], [0 0], 'b-', 'LineWidth', 2);

% 标注偏差值
for i = 1:N
    text(i, devs(i), sprintf('%.3f', devs(i)), ...
         'FontSize', 7, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
end

% X轴标签 = 引脚ID
set(gca, 'XTick', 1:N, 'XTickLabel', arrayfun(@(p) sprintf('%d', p.pin_id), pins, 'Uni', 0));
xlabel('引脚ID', 'FontName', 'SimHei');
ylabel('到座落平面距离(mm)', 'FontName', 'SimHei');
title('(b) 各引脚点云到座落平面的距离分布', ...
      'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;

% 信息标注
text(0.98, 0.98, sprintf('JEDEC共面度: %.4f mm\n座落引脚: %s', ...
     cr.coplanarity_value, num2str(cr.seating_pin_ids)), ...
     'Units', 'normalized', 'FontSize', 9, 'FontName', 'Consolas', ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
     'BackgroundColor', [1 1 0.9], 'EdgeColor', 'k');

sgtitle('图5-11-6: 引脚点云与座落平面（切平面）位置关系', ...
        'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
end

%% ========== 图5-11-7: 翘曲角可视化（引脚拟合平面 vs 座落平面） ==========
function visualize_warp_detail(cr)

pins = cr.pins;
N = length(pins);
seat_n = cr.seating_plane.normal;
seat_d = cr.seating_plane.d;
ref = cr.reference_plane;
warps = cr.warp_angles;

% 选取最多6个引脚展示（按翘曲角降序，展示最严重的）
[~, warp_order] = sort(warps, 'descend');
n_show = min(6, N);
show_idx = warp_order(1:n_show);

n_row = ceil(n_show / 3);
n_col = min(n_show, 3);

figure('Position', [50 50 500*n_col 450*n_row], 'Color', 'w', 'Name', '图5-11-7');

for k = 1:n_show
    i = show_idx(k);
    p = pins(i);
    pc = p.point_cloud;
    n_pin = p.plane_normal;
    pp = p.plane_params;
    mu = mean(pc, 1);
    
    subplot(n_row, n_col, k);
    
    % 拟合平面网格
    xr = [min(pc(:,1)) max(pc(:,1))];
    yr = [min(pc(:,2)) max(pc(:,2))];
    dx = (xr(2)-xr(1))*0.3; dy = (yr(2)-yr(1))*0.3;
    xr = xr + [-dx dx]; yr = yr + [-dy dy];
    [Xg, Yg] = meshgrid(linspace(xr(1),xr(2),20), linspace(yr(1),yr(2),20));
    
    % 引脚拟合平面 (蓝色)
    if abs(pp(3)) > 1e-10
        Zg_pin = -(pp(1)*Xg + pp(2)*Yg + pp(4)) / pp(3);
    else
        Zg_pin = zeros(size(Xg)) + mu(3);
    end
    
    % 座落平面在同一区域的截面 (红色)
    Zg_seat = ref(1)*Xg + ref(2)*Yg + ref(3);
    
    % 绘制两个平面
    h1 = surf(Xg, Yg, Zg_pin, 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
         'FaceColor', [0.3 0.5 1.0]);
    hold on;
    h2 = surf(Xg, Yg, Zg_seat, 'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
         'FaceColor', [1.0 0.3 0.3]);
    
    % 点云
    h3 = scatter3(pc(:,1), pc(:,2), pc(:,3), 10, 'k', 'filled', ...
                  'MarkerFaceAlpha', 0.5);
    
    % 引脚法向量（蓝色）
    arrow_len = max(xr(2)-xr(1), yr(2)-yr(1)) * 0.35;
    quiver3(mu(1), mu(2), mu(3), ...
            n_pin(1)*arrow_len, n_pin(2)*arrow_len, n_pin(3)*arrow_len, ...
            'b', 'LineWidth', 2.5, 'MaxHeadSize', 0.4);
    
    % 座落面法向量（红色）
    quiver3(mu(1), mu(2), mu(3), ...
            seat_n(1)*arrow_len, seat_n(2)*arrow_len, seat_n(3)*arrow_len, ...
            'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.4);
    
    % 绘制夹角弧线
    draw_angle_arc(mu, n_pin, seat_n, arrow_len*0.5, warps(i));
    
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('引脚%d (%s)\n翘曲角=%.2f°  翘曲高=%.4fmm  偏差=%.4fmm', ...
          p.pin_id, p.side, warps(i), cr.warp_heights(i), cr.deviations(i)), ...
          'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimHei');
    
    if k == 1
        legend([h1 h2 h3], {'引脚拟合平面','座落平面','点云'}, ...
               'FontSize', 8, 'FontName', 'SimHei', 'Location', 'best');
    end
    
    axis equal; grid on; view(45, 30);
end

sgtitle(sprintf('图5-11-7: 引脚翘曲角可视化（引脚拟合平面 vs 座落平面）\n按翘曲角降序排列，最大=%.2f°', ...
                max(warps)), ...
        'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');
end

%% ========== 辅助：绘制两个法向量之间的夹角弧线 ==========
function draw_angle_arc(origin, n1, n2, radius, angle_deg)
% 在origin处绘制从n1到n2的弧线，表示夹角
% n1, n2: 单位法向量方向

n1u = n1(:)' / norm(n1);
n2u = n2(:)' / norm(n2);

% 构造弧线所在平面的正交基
v1 = n1u;
v2_raw = n2u - dot(n2u, v1)*v1;
if norm(v2_raw) < 1e-10
    return;  % 两个法向量平行，无需画弧
end
v2 = v2_raw / norm(v2_raw);

% 从n1旋转到n2的圆弧
n_arc = 30;
theta = linspace(0, angle_deg * pi/180, n_arc);
arc_pts = zeros(n_arc, 3);
for j = 1:n_arc
    arc_pts(j,:) = origin + radius * (cos(theta(j))*v1 + sin(theta(j))*v2);
end

plot3(arc_pts(:,1), arc_pts(:,2), arc_pts(:,3), ...
      'm-', 'LineWidth', 2);

% 在弧线中点标注角度值
mid = round(n_arc/2);
text(arc_pts(mid,1), arc_pts(mid,2), arc_pts(mid,3), ...
     sprintf(' %.1f°', angle_deg), ...
     'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.6 0 0.6], ...
     'BackgroundColor', [1 1 1 0.8]);
end
