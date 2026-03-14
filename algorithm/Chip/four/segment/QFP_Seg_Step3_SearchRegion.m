% fig5_10_3.m - 图5-10-3: 基于封装本体边界的四侧引脚精确分割
%
% 核心改进（相比旧版）：
%   1. 搜索区域严格垂直于芯片边缘：
%      利用 fig5_10_1 IRLS 拟合的四条直线和角点，沿各边法线方向
%      构建梯形搜索区域，确保搜索方向与引脚延伸方向一致
%   2. 芯片本体排除：
%      仅排除芯片本体+1像素边界，保留最大引脚根部区域
%   3. 引脚边缘精修（Canny + 分水岭）：
%      粗分割后对每个引脚做 Canny 边缘检测 + 梯度分水岭分割，
%      确保引脚掩膜严格贴合真实边缘
%
% 输入：I_gray    - 原始灰度图像
%       mask_chip - 芯片区域掩膜（来自 fig5_10_1）
%       chip_info - 芯片几何信息（来自 fig5_10_1）
%         .corners    - 4×2 角点 [x,y]（左上/右上/右下/左下）
%         .line_top/bot/left/right - 直线 [a,b,c]
% 输出：pin_regions - 四侧引脚区域结构体
%       mask_pins   - 引脚总掩膜

function [pin_regions, mask_pins] = QFP_Seg_Step3_SearchRegion(I_gray, mask_chip, chip_info)

% 确保 I_gray 是 double 类型
if ~isa(I_gray, 'double')
    I_gray = im2double(I_gray);
end
[img_h, img_w] = size(I_gray);

%% 1. 提取芯片几何参数
corners = chip_info.corners;  % [左上; 右上; 右下; 左下], 每行 [x, y]
L_top   = chip_info.line_top;
L_bot   = chip_info.line_bot;
L_left  = chip_info.line_left;
L_right = chip_info.line_right;

% 芯片中心
chip_cx = mean(corners(:,1));
chip_cy = mean(corners(:,2));

% 各边长度（用于自适应参数）
edge_top_len  = norm(corners(2,:) - corners(1,:));
edge_bot_len  = norm(corners(3,:) - corners(4,:));
edge_left_len = norm(corners(4,:) - corners(1,:));
edge_right_len = norm(corners(3,:) - corners(2,:));
chip_short = min(mean([edge_top_len, edge_bot_len]), mean([edge_left_len, edge_right_len]));

fprintf('芯片角点:\n');
corner_names = {'左上', '右上', '右下', '左下'};
for i = 1:4
    fprintf('  %s: (%.1f, %.1f)\n', corner_names{i}, corners(i,1), corners(i,2));
end

%% 2. 计算引脚搜索参数
L_pin_max = round(chip_short * 0.3);
L_pin_max = max(L_pin_max, 30);
fprintf('引脚最大延伸长度: %d 像素\n', L_pin_max);

%% 3. 构建严格垂直于芯片边缘的搜索区域
% 对每条边，沿其法线方向（朝外）扩展 L_pin_max 像素构成梯形搜索区域
% 法线方向由直线 ax+by+c=0 的 [a,b] 确定，朝向芯片外部

fprintf('构建严格垂直于芯片边缘的搜索区域...\n');

overlap = round(L_pin_max * 0.15);  % 向芯片内部的余量
overlap = max(overlap, 8);

% 上侧搜索区域：corners(1)→corners(2)
mask_top = build_edge_search_mask(img_h, img_w, ...
    corners(1,:), corners(2,:), L_top, chip_cx, chip_cy, ...
    L_pin_max, overlap);

% 下侧搜索区域：corners(4)→corners(3)
mask_bottom = build_edge_search_mask(img_h, img_w, ...
    corners(4,:), corners(3,:), L_bot, chip_cx, chip_cy, ...
    L_pin_max, overlap);

% 左侧搜索区域：corners(1)→corners(4)
mask_left = build_edge_search_mask(img_h, img_w, ...
    corners(1,:), corners(4,:), L_left, chip_cx, chip_cy, ...
    L_pin_max, overlap);

% 右侧搜索区域：corners(2)→corners(3)
mask_right = build_edge_search_mask(img_h, img_w, ...
    corners(2,:), corners(3,:), L_right, chip_cx, chip_cy, ...
    L_pin_max, overlap);

%% 4. 排除芯片本体（仅排除芯片本体 + 1像素边界）
% 只排除芯片边缘1像素，保留尽可能多的引脚根部区域
edge_exclude_width = 1;  % 排除带宽度（像素）
se_edge = strel('disk', edge_exclude_width);
mask_chip_dilated = imdilate(mask_chip, se_edge);

% 从搜索区域中排除芯片本体（+1px边界）
mask_top    = mask_top    & ~mask_chip_dilated;
mask_bottom = mask_bottom & ~mask_chip_dilated;
mask_left   = mask_left   & ~mask_chip_dilated;
mask_right  = mask_right  & ~mask_chip_dilated;

fprintf('搜索区域面积: 上=%d, 下=%d, 左=%d, 右=%d 像素\n', ...
    sum(mask_top(:)), sum(mask_bottom(:)), sum(mask_left(:)), sum(mask_right(:)));
fprintf('芯片边缘排除宽度: %d 像素\n', edge_exclude_width);

%% 5. 构建ROI包围矩形（用于子图提取）
rects_side = cell(4,1);
masks_side = {mask_top, mask_bottom, mask_left, mask_right};
for s = 1:4
    [rows_s, cols_s] = find(masks_side{s});
    if isempty(rows_s)
        rects_side{s} = struct('y1', 1, 'y2', 2, 'x1', 1, 'x2', 2);
    else
        rects_side{s} = struct('y1', max(1, min(rows_s)-5), ...
                               'y2', min(img_h, max(rows_s)+5), ...
                               'x1', max(1, min(cols_s)-5), ...
                               'x2', min(img_w, max(cols_s)+5));
    end
end

%% 6. 预计算全图特征（纹理 + 梯度）
fprintf('计算纹理和梯度特征...\n');

% 梯度幅值
I_smooth = imgaussfilt(I_gray, 1.0);
G_mag = imgradient(I_smooth, 'sobel');
G_norm = G_mag / (max(G_mag(:)) + eps);

% 局部标准差（纹理能量）
nhood = true(9);
I_std = stdfilt(I_gray, nhood);
I_std_norm = I_std / (max(I_std(:)) + eps);

% 融合特征
F_combined = 0.6 * I_std_norm + 0.4 * G_norm;
F_combined = F_combined / (max(F_combined(:)) + eps);

fprintf('  纹理能量范围: [%.4f, %.4f]\n', min(I_std_norm(:)), max(I_std_norm(:)));
fprintf('  融合特征范围: [%.4f, %.4f]\n', min(F_combined(:)), max(F_combined(:)));

%% 7. 四侧引脚分割
fprintf('\n开始四侧引脚分割...\n');

sides = {'上', '下', '左', '右'};
dirs = {'horizontal', 'horizontal', 'vertical', 'vertical'};

mask_pins_all = cell(4,1);
pins_all      = cell(4,1);
debug_all     = cell(4,1);

for s = 1:4
    fprintf('  分割%s侧引脚...\n', sides{s});
    [mask_pins_all{s}, pins_all{s}, debug_all{s}] = ...
        segment_pins_texture(I_gray, F_combined, I_std_norm, G_norm, ...
                             masks_side{s}, rects_side{s}, dirs{s});
    fprintf('    检测到 %d 个引脚\n', length(pins_all{s}));
end

%% 8. 引脚边缘精修
fprintf('\n引脚边缘精修...\n');

% 保存粗分割结果用于可视化对比
mask_pins_coarse = cell(4,1);
pins_coarse = cell(4,1);
for s = 1:4
    mask_pins_coarse{s} = mask_pins_all{s};
    pins_coarse{s} = pins_all{s};
end

for s = 1:4
    if isempty(pins_all{s})
        continue;
    end
    fprintf('  精修%s侧 %d 个引脚...\n', sides{s}, length(pins_all{s}));
    [mask_pins_all{s}, pins_all{s}] = refine_pin_edges( ...
        I_gray, G_mag, mask_pins_all{s}, pins_all{s}, mask_chip_dilated);
end

% 汇总
mask_pins_top    = mask_pins_all{1};
mask_pins_bottom = mask_pins_all{2};
mask_pins_left   = mask_pins_all{3};
mask_pins_right  = mask_pins_all{4};

pins_top    = pins_all{1};
pins_bottom = pins_all{2};
pins_left   = pins_all{3};
pins_right  = pins_all{4};

fprintf('\n引脚分割完成: 上=%d, 下=%d, 左=%d, 右=%d\n', ...
    length(pins_top), length(pins_bottom), length(pins_left), length(pins_right));

%% 9. 构造输出结构
pin_regions.top.num_pins    = length(pins_top);
pin_regions.top.pins        = pins_top;
pin_regions.top.mask_pins   = mask_pins_top;

pin_regions.bottom.num_pins = length(pins_bottom);
pin_regions.bottom.pins     = pins_bottom;
pin_regions.bottom.mask_pins = mask_pins_bottom;

pin_regions.left.num_pins   = length(pins_left);
pin_regions.left.pins       = pins_left;
pin_regions.left.mask_pins  = mask_pins_left;

pin_regions.right.num_pins  = length(pins_right);
pin_regions.right.pins      = pins_right;
pin_regions.right.mask_pins = mask_pins_right;

mask_pins = mask_pins_top | mask_pins_bottom | mask_pins_left | mask_pins_right;

% 最终安全保障：确保引脚掩膜绝不包含芯片本体区域
mask_pins = mask_pins & ~mask_chip_dilated;

total_pins = pin_regions.top.num_pins + pin_regions.bottom.num_pins + ...
             pin_regions.left.num_pins + pin_regions.right.num_pins;

%% 10. 可视化 Figure 1: 引脚搜索区域构建流程（论文图 — 2×3 布局）
%  展示从芯片边界识别到引脚搜索区域构建的关键步骤
bnd_chip = bwboundaries(mask_chip);
mask_all_search = mask_top | mask_bottom | mask_left | mask_right;

% 汇总粗分割掩膜
mask_coarse_all = false(img_h, img_w);
for s = 1:4
    mask_coarse_all = mask_coarse_all | mask_pins_coarse{s};
end

figure('Position', [50, 100, 1200, 750], 'Color', 'w');

% (a) 原始灰度图像
subplot(2,3,1);
imshow(I_gray);
% title('(a) 原始灰度图像', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (b) 芯片本体边界与IRLS拟合边缘
subplot(2,3,2);
imshow(I_gray); hold on;
for k = 1:length(bnd_chip)
    plot(bnd_chip{k}(:,2), bnd_chip{k}(:,1), 'g-', 'LineWidth', 2);
end
plot(corners(:,1), corners(:,2), 'r+', 'MarkerSize', 14, 'LineWidth', 2.5);
% 绘制IRLS拟合直线（角点之间的连线）
line_pairs = [1 2; 4 3; 1 4; 2 3];  % 上、下、左、右
line_colors_fig = {'r', 'r', 'b', 'b'};
for lp = 1:4
    p1 = corners(line_pairs(lp,1), :);
    p2 = corners(line_pairs(lp,2), :);
    plot([p1(1) p2(1)], [p1(2) p2(2)], '--', 'Color', line_colors_fig{lp}, 'LineWidth', 1.5);
end
% title('(b) 芯片边界与IRLS拟合线', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (c) 垂直于芯片边缘的搜索区域
subplot(2,3,3);
imshow(I_gray); hold on;
% 按侧分色半透明叠加
overlay_tb = zeros(img_h, img_w, 3);
overlay_tb(:,:,1) = double(mask_top | mask_bottom);
overlay_tb(:,:,2) = double(mask_top | mask_bottom) * 0.3;
mask_tb = mask_top | mask_bottom;
h_tb = imshow(overlay_tb); set(h_tb, 'AlphaData', mask_tb * 0.4);

overlay_lr = zeros(img_h, img_w, 3);
overlay_lr(:,:,2) = double(mask_left | mask_right);
overlay_lr(:,:,3) = double(mask_left | mask_right) * 0.5;
mask_lr = mask_left | mask_right;
h_lr = imshow(overlay_lr); set(h_lr, 'AlphaData', mask_lr * 0.4);

for k = 1:length(bnd_chip)
    plot(bnd_chip{k}(:,2), bnd_chip{k}(:,1), 'g-', 'LineWidth', 1.5);
end
% 绘制法线方向箭头示意
lines_all = {L_top, L_bot, L_left, L_right};
pts1_all = {corners(1,:), corners(4,:), corners(1,:), corners(2,:)};
pts2_all = {corners(2,:), corners(3,:), corners(4,:), corners(3,:)};
for lp = 1:4
    a_l = lines_all{lp}(1); b_l = lines_all{lp}(2);
    nl = sqrt(a_l^2+b_l^2)+eps;
    nxl = a_l/nl; nyl = b_l/nl;
    mid_l = (pts1_all{lp} + pts2_all{lp})/2;
    test_l = mid_l + [nxl nyl]*10;
    if (test_l(1)-chip_cx)^2+(test_l(2)-chip_cy)^2 < (mid_l(1)-chip_cx)^2+(mid_l(2)-chip_cy)^2
        nxl = -nxl; nyl = -nyl;
    end
    quiver(mid_l(1), mid_l(2), nxl*20, nyl*20, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
end
% title(sprintf('(c) 搜索区域(L=%dpx)', L_pin_max), 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (d) 纹理能量图（搜索区域内）
subplot(2,3,4);
I_std_vis = I_std_norm;
I_std_vis(~mask_all_search) = 0;
imshow(I_std_vis, []); colormap(gca, 'hot');
% title('(d) 纹理能量(stdfilt 9×9)', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (e) 融合特征图（纹理0.6 + 梯度0.4）
subplot(2,3,5);
F_vis = F_combined;
F_vis(~mask_all_search) = 0;
imshow(F_vis, []); colormap(gca, 'hot');
% title('(e) 融合特征(0.6T+0.4G)', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (f) 粗分割结果
subplot(2,3,6);
imshow(I_gray); hold on;
for k = 1:length(bnd_chip)
    plot(bnd_chip{k}(:,2), bnd_chip{k}(:,1), 'g-', 'LineWidth', 1.5);
end
bnd_coarse = bwboundaries(mask_coarse_all);
for k = 1:length(bnd_coarse)
    plot(bnd_coarse{k}(:,2), bnd_coarse{k}(:,1), 'r-', 'LineWidth', 1.5);
end
overlay_coarse = cat(3, double(mask_coarse_all), double(mask_coarse_all)*0.6, zeros(img_h,img_w));
h_c = imshow(overlay_coarse); set(h_c, 'AlphaData', mask_coarse_all * 0.35);
% title(sprintf('(f) 粗分割结果(%d个引脚)', total_pins), 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

sgtitle('引脚搜索区域构建与特征提取流程', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

%% 11. 可视化 Figure 2: 引脚边缘精修效果对比（论文图 — 2×3 布局）
%  展示粗分割 vs 精修后的边缘对比，突出精修效果
figure('Position', [80, 60, 1200, 750], 'Color', 'w');

% (a) 粗分割掩膜（全图）
subplot(2,3,1);
imshow(I_gray); hold on;
for k = 1:length(bnd_chip)
    plot(bnd_chip{k}(:,2), bnd_chip{k}(:,1), 'g-', 'LineWidth', 1.5);
end
bnd_coarse2 = bwboundaries(mask_coarse_all);
for k = 1:length(bnd_coarse2)
    plot(bnd_coarse2{k}(:,2), bnd_coarse2{k}(:,1), 'y-', 'LineWidth', 1.5);
end
overlay_cr = cat(3, double(mask_coarse_all), double(mask_coarse_all)*0.7, zeros(img_h,img_w));
h_cr = imshow(overlay_cr); set(h_cr, 'AlphaData', mask_coarse_all * 0.3);
% title('(a) 粗分割结果', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (b) 精修后掩膜（全图）
subplot(2,3,2);
imshow(I_gray); hold on;
for k = 1:length(bnd_chip)
    plot(bnd_chip{k}(:,2), bnd_chip{k}(:,1), 'g-', 'LineWidth', 1.5);
end
bnd_refined = bwboundaries(mask_pins);
for k = 1:length(bnd_refined)
    plot(bnd_refined{k}(:,2), bnd_refined{k}(:,1), 'r-', 'LineWidth', 1.5);
end
overlay_ref = cat(3, double(mask_pins), double(mask_pins)*0.5, zeros(img_h,img_w));
h_ref = imshow(overlay_ref); set(h_ref, 'AlphaData', mask_pins * 0.3);
% title('(b) 边缘精修后结果', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (c) 粗分割 vs 精修叠加对比（全图）
subplot(2,3,3);
imshow(I_gray); hold on;
for k = 1:length(bnd_chip)
    plot(bnd_chip{k}(:,2), bnd_chip{k}(:,1), 'g-', 'LineWidth', 1);
end
% 粗分割边界（黄色虚线）
for k = 1:length(bnd_coarse2)
    plot(bnd_coarse2{k}(:,2), bnd_coarse2{k}(:,1), 'y--', 'LineWidth', 1.5);
end
% 精修边界（红色实线）
for k = 1:length(bnd_refined)
    plot(bnd_refined{k}(:,2), bnd_refined{k}(:,1), 'r-', 'LineWidth', 1.5);
end
% title('(c) 对比: 粗(黄虚)→精(红实)', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (d)-(f) 选取有引脚的侧做局部放大对比
%   找到引脚数最多的两侧做详细展示
pin_counts = [length(pins_top), length(pins_bottom), length(pins_left), length(pins_right)];
[~, sort_idx] = sort(pin_counts, 'descend');
% 取引脚数最多的两侧（至少有引脚）
detail_sides = sort_idx(pin_counts(sort_idx) > 0);
if length(detail_sides) > 3
    detail_sides = detail_sides(1:3);
end
if isempty(detail_sides)
    detail_sides = [3, 4];  % 默认左右
end

side_labels = {'上侧', '下侧', '左侧', '右侧'};
for di = 1:min(3, length(detail_sides))
    s = detail_sides(di);
    r = rects_side{s};
    
    subplot(2,3,3+di);
    I_roi_detail = I_gray(r.y1:r.y2, r.x1:r.x2);
    imshow(I_roi_detail); hold on;
    
    % 粗分割边界（黄色虚线）
    mask_coarse_roi = mask_pins_coarse{s}(r.y1:r.y2, r.x1:r.x2);
    bnd_cr_roi = bwboundaries(mask_coarse_roi);
    for k = 1:length(bnd_cr_roi)
        plot(bnd_cr_roi{k}(:,2), bnd_cr_roi{k}(:,1), 'y--', 'LineWidth', 2);
    end
    
    % 精修边界（红色实线）
    mask_refined_roi = mask_pins_all{s}(r.y1:r.y2, r.x1:r.x2);
    bnd_rf_roi = bwboundaries(mask_refined_roi);
    for k = 1:length(bnd_rf_roi)
        plot(bnd_rf_roi{k}(:,2), bnd_rf_roi{k}(:,1), 'r-', 'LineWidth', 2);
    end
    
    % 半透明叠加精修区域
    overlay_detail = cat(3, double(mask_refined_roi), double(mask_refined_roi)*0.4, zeros(size(mask_refined_roi)));
    h_dt = imshow(overlay_detail); set(h_dt, 'AlphaData', mask_refined_roi * 0.25);
    
    n_pins_side = length(pins_all{s});
    % title(sprintf('(%c) %s局部放大(%d引脚)', 'c'+di, side_labels{s}, n_pins_side), ...
    %       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
end

sgtitle('引脚边缘精修效果对比（Canny边缘+梯度分水岭）', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

%% 12. 可视化 Figure 3: 各侧引脚分割处理流程细节（论文图 — N×4 布局）
%  仅展示有引脚的侧，每侧一行，4步流程
active_sides = find(pin_counts > 0);
n_active = length(active_sides);
if n_active == 0
    n_active = 4; active_sides = 1:4;
end

figure('Position', [100, 40, 1200, 220*n_active+80], 'Color', 'w');

for ai = 1:n_active
    s = active_sides(ai);
    dbg = debug_all{s};
    r = rects_side{s};
    
    % 第1列：搜索区域内原始ROI
    subplot(n_active, 4, (ai-1)*4+1);
    imshow(dbg.I_roi);
    % 叠加搜索区域掩膜轮廓
    mask_search_roi = masks_side{s}(r.y1:r.y2, r.x1:r.x2);
    bnd_search = bwboundaries(mask_search_roi);
    hold on;
    for k = 1:length(bnd_search)
        plot(bnd_search{k}(:,2), bnd_search{k}(:,1), 'c-', 'LineWidth', 1);
    end
    % title(sprintf('(%c) %s: 搜索ROI', char('a'+(ai-1)*4), side_labels{s}), ...
    %       'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimHei');
    
    % 第2列：融合特征热力图
    subplot(n_active, 4, (ai-1)*4+2);
    F_roi_vis = F_combined(r.y1:r.y2, r.x1:r.x2) .* mask_search_roi;
    imshow(F_roi_vis, []); colormap(gca, 'hot');
    % title(sprintf('(%c) 融合特征', char('b'+(ai-1)*4)), ...
    %       'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimHei');
    
    % 第3列：Otsu阈值分割 + 形态学
    subplot(n_active, 4, (ai-1)*4+3);
    % 将阈值和形态学结果用不同颜色叠加
    I_roi_rgb = repmat(dbg.I_roi, [1 1 3]);
    % 阈值分割轮廓（蓝色）
    bnd_thresh = bwboundaries(dbg.mask_thresh_roi);
    % 形态学后轮廓（绿色）
    bnd_morph = bwboundaries(dbg.mask_morph_roi);
    imshow(I_roi_rgb); hold on;
    for k = 1:length(bnd_thresh)
        plot(bnd_thresh{k}(:,2), bnd_thresh{k}(:,1), 'b-', 'LineWidth', 1);
    end
    for k = 1:length(bnd_morph)
        plot(bnd_morph{k}(:,2), bnd_morph{k}(:,1), 'g-', 'LineWidth', 1.5);
    end
    overlay_morph = zeros(size(dbg.I_roi,1), size(dbg.I_roi,2), 3);
    overlay_morph(:,:,2) = double(dbg.mask_morph_roi);
    h_om = imshow(overlay_morph); set(h_om, 'AlphaData', dbg.mask_morph_roi * 0.3);
    % title(sprintf('(%c) 阈值(蓝)+形态学(绿)\nT=%.3f', char('c'+(ai-1)*4), dbg.T_texture), ...
    %       'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimHei');
    
    % 第4列：精修后最终引脚
    subplot(n_active, 4, (ai-1)*4+4);
    imshow(dbg.I_roi); hold on;
    mask_final_roi = mask_pins_all{s}(r.y1:r.y2, r.x1:r.x2);
    mask_coarse_s_roi = mask_pins_coarse{s}(r.y1:r.y2, r.x1:r.x2);
    
    % 粗分割边界（黄虚线）
    bnd_cr2 = bwboundaries(mask_coarse_s_roi);
    for k = 1:length(bnd_cr2)
        plot(bnd_cr2{k}(:,2), bnd_cr2{k}(:,1), 'y--', 'LineWidth', 1.5);
    end
    % 精修边界（红实线）
    bnd_rf2 = bwboundaries(mask_final_roi);
    for k = 1:length(bnd_rf2)
        plot(bnd_rf2{k}(:,2), bnd_rf2{k}(:,1), 'r-', 'LineWidth', 2);
    end
    overlay_fin = cat(3, double(mask_final_roi), double(mask_final_roi)*0.4, zeros(size(mask_final_roi)));
    h_of = imshow(overlay_fin); set(h_of, 'AlphaData', mask_final_roi * 0.25);
    % title(sprintf('(%c) 精修引脚: %d个', char('d'+(ai-1)*4), length(pins_all{s})), ...
    %       'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimHei');
end

sgtitle('各侧引脚分割处理流程（搜索ROI→特征→分割→精修）', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

%% 13. 可视化 Figure 4: 最终引脚定位结果（论文图 — 1×2 布局）
figure('Position', [130, 80, 1100, 500], 'Color', 'w');

% (a) 引脚掩膜按侧着色 + 芯片边界
subplot(1,2,1);
imshow(I_gray); hold on;
for k = 1:length(bnd_chip)
    plot(bnd_chip{k}(:,2), bnd_chip{k}(:,1), 'g-', 'LineWidth', 2);
end
colors_paper = {[1 0.2 0.2], [0.2 0.8 0.2], [0.8 0.2 0.8], [0.2 0.8 0.8]};
side_legend = {};
for s = 1:4
    if sum(mask_pins_all{s}(:)) > 0
        overlay_ps = zeros(img_h, img_w, 3);
        for c = 1:3, overlay_ps(:,:,c) = double(mask_pins_all{s}) * colors_paper{s}(c); end
        h_ps = imshow(overlay_ps); set(h_ps, 'AlphaData', mask_pins_all{s} * 0.5);
        % 绘制引脚边界
        bnd_ps = bwboundaries(mask_pins_all{s});
        for k = 1:length(bnd_ps)
            plot(bnd_ps{k}(:,2), bnd_ps{k}(:,1), '-', 'Color', colors_paper{s}, 'LineWidth', 1.5);
        end
    end
end
% title(sprintf('(a) 引脚分侧着色\n上%d  下%d  左%d  右%d', ...
%       length(pins_top), length(pins_bottom), length(pins_left), length(pins_right)), ...
%       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (b) 最终结果：边界框 + 引脚编号 + 统计
subplot(1,2,2);
imshow(I_gray); hold on;
for k = 1:length(bnd_chip)
    plot(bnd_chip{k}(:,2), bnd_chip{k}(:,1), 'g-', 'LineWidth', 2);
end
% 绘制引脚边界
bnd_final = bwboundaries(mask_pins);
for k = 1:length(bnd_final)
    plot(bnd_final{k}(:,2), bnd_final{k}(:,1), 'r-', 'LineWidth', 1.5);
end
% 绘制边界框和引脚编号
side_colors_paper = {'r', [0 0.7 0], [0.7 0 0.7], [0 0.7 0.7]};
all_pins_flat = {};
all_sides_flat = [];
for s = 1:4
    for pi = 1:length(pins_all{s})
        all_pins_flat{end+1} = pins_all{s}(pi);
        all_sides_flat(end+1) = s;
    end
end
pin_global_idx = 0;
for s = 1:4
    for pi = 1:length(pins_all{s})
        pin_global_idx = pin_global_idx + 1;
        bb_p = pins_all{s}(pi).bbox;
        rectangle('Position', bb_p, 'EdgeColor', side_colors_paper{s}, 'LineWidth', 1.5);
        % 引脚编号标注
        text(bb_p(1)+bb_p(3)/2, bb_p(2)+bb_p(4)/2, sprintf('%d', pin_global_idx), ...
             'FontSize', 8, 'FontWeight', 'bold', 'Color', 'w', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'BackgroundColor', [0 0 0 0.5]);
    end
end
% 四侧引脚数标注
text_props2 = {'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei', ...
               'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.6], 'Color', 'y'};
text(chip_cx, min(corners(:,2))-18, sprintf('上: %d', length(pins_top)), text_props2{:});
text(chip_cx, max(corners(:,2))+18, sprintf('下: %d', length(pins_bottom)), text_props2{:});
text(min(corners(:,1))-25, chip_cy, sprintf('左: %d', length(pins_left)), text_props2{:});
text(max(corners(:,1))+25, chip_cy, sprintf('右: %d', length(pins_right)), text_props2{:});
% title(sprintf('(b) 最终引脚定位结果\n共%d个引脚, %d像素', total_pins, sum(mask_pins(:))), ...
%       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

sgtitle('QFP芯片引脚精确定位结果', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

end

%% ====================================================================
%%  辅助函数
%% ====================================================================

function mask = build_edge_search_mask(img_h, img_w, pt1, pt2, line_params, cx, cy, L_pin, overlap)
% 构建严格垂直于芯片边缘的搜索区域掩膜
%
% 思路：沿芯片边缘 pt1→pt2 的法线方向（朝外），扩展 L_pin 像素
%       同时向芯片内部扩展 overlap 像素（覆盖引脚根部）
%       最终得到一个平行四边形搜索区域
%
% 输入：img_h, img_w - 图像尺寸
%       pt1, pt2 - 边上的两个角点 [x, y]
%       line_params - 该边直线 [a, b, c] (ax+by+c=0)
%       cx, cy - 芯片中心（用于确定法线朝外方向）
%       L_pin - 向外搜索距离
%       overlap - 向内搜索距离

    a = line_params(1); b = line_params(2);
    
    % 法线方向（单位化）
    n_len = sqrt(a^2 + b^2) + eps;
    nx = a / n_len;
    ny = b / n_len;
    
    % 确保法线朝向芯片外部（远离芯片中心）
    mid_x = (pt1(1) + pt2(1)) / 2;
    mid_y = (pt1(2) + pt2(2)) / 2;
    
    % 测试法线方向：中点沿法线偏移，应远离芯片中心
    test_x = mid_x + nx * 10;
    test_y = mid_y + ny * 10;
    d_to_center_orig = (mid_x - cx)^2 + (mid_y - cy)^2;
    d_to_center_test = (test_x - cx)^2 + (test_y - cy)^2;
    
    if d_to_center_test < d_to_center_orig
        % 法线朝内，翻转
        nx = -nx;
        ny = -ny;
    end
    
    % 边切线方向的扩展余量
    tx = pt2(1) - pt1(1);
    ty = pt2(2) - pt1(2);
    t_len = sqrt(tx^2 + ty^2) + eps;
    tx = tx / t_len;
    ty = ty / t_len;
    
    % 搜索区域四个顶点
    inner_offset_x = -nx * overlap;
    inner_offset_y = -ny * overlap;
    outer_offset_x = nx * L_pin;
    outer_offset_y = ny * L_pin;
    
    margin_x = tx * overlap;
    margin_y = ty * overlap;
    
    poly_x = [pt1(1) + inner_offset_x - margin_x, ...
              pt2(1) + inner_offset_x + margin_x, ...
              pt2(1) + outer_offset_x + margin_x, ...
              pt1(1) + outer_offset_x - margin_x];
    poly_y = [pt1(2) + inner_offset_y - margin_y, ...
              pt2(2) + inner_offset_y + margin_y, ...
              pt2(2) + outer_offset_y + margin_y, ...
              pt1(2) + outer_offset_y - margin_y];
    
    % 裁剪到图像范围
    poly_x = max(1, min(img_w, poly_x));
    poly_y = max(1, min(img_h, poly_y));
    
    % 生成掩膜
    mask = poly2mask(poly_x, poly_y, img_h, img_w);
end

function [mask_pins_side, pins, dbg] = segment_pins_texture(I_gray, F_combined, I_std_norm, ~, mask_search, roi_rect, direction)
% 基于纹理+梯度双特征的引脚分割（搜索掩膜已排除芯片边缘带）

    [img_h, img_w] = size(I_gray);
    mask_pins_side = false(img_h, img_w);
    pins = [];
    
    r = roi_rect;
    I_roi = I_gray(r.y1:r.y2, r.x1:r.x2);
    mask_roi = mask_search(r.y1:r.y2, r.x1:r.x2);
    F_roi = F_combined(r.y1:r.y2, r.x1:r.x2);
    T_roi = I_std_norm(r.y1:r.y2, r.x1:r.x2);
    [roi_h, roi_w] = size(I_roi);
    
    % 初始化 debug
    dbg.I_roi = I_roi;
    dbg.texture_roi = T_roi .* mask_roi;
    dbg.T_texture = 0;
    dbg.mask_thresh_roi = false(roi_h, roi_w);
    dbg.mask_morph_roi  = false(roi_h, roi_w);
    dbg.mask_threshold  = false(img_h, img_w);
    dbg.mask_morphed    = false(img_h, img_w);
    
    if sum(mask_roi(:)) < 100
        fprintf('    搜索区域过小(%d px)，跳过\n', sum(mask_roi(:)));
        return;
    end
    
    % ── Step 1: 对融合特征做自适应阈值 ──
    valid_F = F_roi(mask_roi);
    T_texture = graythresh(valid_F);
    
    mask_high_texture = (F_roi > T_texture) & mask_roi;
    
    fprintf('    纹理阈值: T=%.4f, 高纹理像素: %d\n', T_texture, sum(mask_high_texture(:)));
    
    dbg.T_texture = T_texture;
    dbg.mask_thresh_roi = mask_high_texture;
    dbg.mask_threshold = false(img_h, img_w);
    dbg.mask_threshold(r.y1:r.y2, r.x1:r.x2) = mask_high_texture;
    
    % ── Step 2: 方向性形态学精修 ──
    if strcmp(direction, 'horizontal')
        se_close1 = strel('line', 12, 90);
        se_close2 = strel('line', 5, 0);
    else
        se_close1 = strel('line', 12, 0);
        se_close2 = strel('line', 5, 90);
    end
    
    mask_step = imclose(mask_high_texture, se_close1);
    mask_step = imclose(mask_step, se_close2);
    mask_step = imfill(mask_step, 'holes');
    
    se_open = strel('disk', 3);
    mask_step = imopen(mask_step, se_open);
    mask_step = imfill(mask_step, 'holes');
    
    dbg.mask_morph_roi = mask_step;
    dbg.mask_morphed = false(img_h, img_w);
    dbg.mask_morphed(r.y1:r.y2, r.x1:r.x2) = mask_step;
    
    % ── Step 3: 连通域分析 + 几何筛选 ──
    CC = bwconncomp(mask_step);
    if CC.NumObjects == 0
        return;
    end
    stats = regionprops(CC, 'Area', 'BoundingBox', 'Centroid', 'Solidity', ...
                        'MajorAxisLength', 'MinorAxisLength');
    
    roi_area = sum(mask_roi(:));
    min_area = max(50, roi_area * 0.003);
    max_area = roi_area * 0.20;
    
    fprintf('    连通域: %d, 面积范围[%.0f, %.0f]\n', CC.NumObjects, min_area, max_area);
    
    pin_count = 0;
    for i = 1:CC.NumObjects
        area_i = stats(i).Area;
        bbox_i = stats(i).BoundingBox;
        solidity_i = stats(i).Solidity;
        centroid_i = stats(i).Centroid;
        major = stats(i).MajorAxisLength;
        minor = stats(i).MinorAxisLength;
        
        if area_i < min_area || area_i > max_area, continue; end
        if solidity_i < 0.4, continue; end
        elongation = major / (minor + eps);
        if elongation > 10, continue; end
        if bbox_i(3) < 5 || bbox_i(4) < 5, continue; end
        
        abs_cx = centroid_i(1) + r.x1 - 1;
        abs_cy = centroid_i(2) + r.y1 - 1;
        
        pin_count = pin_count + 1;
        pins(pin_count).area     = area_i;
        pins(pin_count).bbox     = [bbox_i(1)+r.x1-1, bbox_i(2)+r.y1-1, bbox_i(3), bbox_i(4)];
        pins(pin_count).centroid = [abs_cx, abs_cy];
        pins(pin_count).solidity = solidity_i;
        
        % 写入全图掩膜
        [sub_r, sub_c] = ind2sub([roi_h, roi_w], CC.PixelIdxList{i});
        for p = 1:length(sub_r)
            mask_pins_side(sub_r(p)+r.y1-1, sub_c(p)+r.x1-1) = true;
        end
        
        fprintf('    引脚#%d: 面积=%d, 中心=(%.0f,%.0f), 实心度=%.2f, 伸长比=%.1f\n', ...
                pin_count, area_i, abs_cx, abs_cy, solidity_i, elongation);
    end
end

function [mask_refined, pins_refined] = refine_pin_edges(I_gray, G_mag, mask_pins, pins, mask_chip_exclude)
% 引脚边缘精修：使用 Canny 边缘 + 梯度分水岭方法获得精确引脚边界
%
% 思路：
%   1. 取引脚 bbox 扩展区域
%   2. 在局部区域做 Canny 边缘检测，获取引脚真实轮廓
%   3. 利用梯度幅值构建分水岭势场，以粗掩膜为前景种子、
%      远离引脚区域为背景种子，做标记分水岭分割
%   4. 分水岭结果与梯度约束 imreconstruct 取交集，
%      保证最终掩膜严格贴合引脚边缘
%   5. 全程排除芯片本体区域，防止掩膜渗透到芯片边缘

    [img_h, img_w] = size(I_gray);
    mask_refined = false(img_h, img_w);
    pins_refined = pins;
    
    if isempty(pins)
        return;
    end
    
    expand_px = 12;  % bbox 扩展像素（增大以包含完整边缘）
    
    for i = 1:length(pins)
        bb = pins(i).bbox;  % [x, y, w, h]
        
        % 扩展 bbox
        rx1 = max(1, round(bb(1)) - expand_px);
        ry1 = max(1, round(bb(2)) - expand_px);
        rx2 = min(img_w, round(bb(1) + bb(3)) + expand_px);
        ry2 = min(img_h, round(bb(2) + bb(4)) + expand_px);
        
        I_local = I_gray(ry1:ry2, rx1:rx2);
        G_local = G_mag(ry1:ry2, rx1:rx2);
        mask_local = mask_pins(ry1:ry2, rx1:rx2);
        mask_chip_local = mask_chip_exclude(ry1:ry2, rx1:rx2);
        [lh, lw] = size(I_local);
        
        % ── Step 1: Canny 边缘检测 ──
        % 自适应阈值 Canny
        edges_canny = edge(I_local, 'canny');
        
        % 同时用梯度幅值生成强边缘掩膜
        G_local_norm = G_local / (max(G_local(:)) + eps);
        
        % ── Step 2: 梯度分水岭分割 ──
        % 使用梯度幅值作为分水岭势场
        G_watershed = imgaussfilt(G_local_norm, 0.8);  % 轻微平滑避免过度分割
        
        % 前景种子：侵蚀粗掩膜（确保在引脚内部），且必须在芯片外
        se_fg = strel('disk', 2);
        mask_fg_seed = imerode(mask_local, se_fg) & ~mask_chip_local;
        if sum(mask_fg_seed(:)) < 10
            se_fg = strel('disk', 1);
            mask_fg_seed = imerode(mask_local, se_fg) & ~mask_chip_local;
        end
        if sum(mask_fg_seed(:)) < 5
            mask_fg_seed = mask_local & ~mask_chip_local;
        end
        
        % 背景种子：距离粗掩膜边界远处的像素 + 芯片本体区域
        mask_bg_seed = ~imdilate(mask_local, strel('disk', 4));
        mask_bg_seed = mask_bg_seed | mask_chip_local;  % 芯片本体强制为背景
        % 确保背景种子不为空
        if sum(mask_bg_seed(:)) < 10
            mask_bg_seed = false(lh, lw);
            mask_bg_seed([1,lh], :) = true;
            mask_bg_seed(:, [1,lw]) = true;
        end
        
        % 标记矩阵
        markers = zeros(lh, lw);
        markers(mask_fg_seed) = 1;  % 前景
        markers(mask_bg_seed) = 2;  % 背景
        
        % 对梯度势场施加标记，做分水岭
        G_ws = imimposemin(G_watershed, markers > 0);
        L_ws = watershed(G_ws);
        mask_ws = (L_ws == 1);  % 前景区域
        mask_ws = mask_ws & ~mask_chip_local;  % 排除芯片区域
        
        % ── Step 3: Canny边缘约束精修 ──
        % 从粗掩膜向外扩展，但不越过 Canny 边缘线
        % 先将 Canny 边缘加粗1px作为屏障
        edges_barrier = imdilate(edges_canny, strel('disk', 1));
        % 芯片边缘也作为不可逾越的屏障
        edges_barrier = edges_barrier | mask_chip_local;
        
        % 条件膨胀：从种子出发，不越过边缘屏障（也不进入芯片区域）
        se_expand = strel('disk', 3);
        mask_expand_limit = imdilate(mask_local, se_expand) & ~mask_chip_local;
        mask_reachable = mask_expand_limit & ~edges_barrier;
        mask_reachable = mask_reachable | mask_fg_seed;  % 种子必须可达
        
        mask_edge_recon = imreconstruct(mask_fg_seed, mask_reachable);
        mask_edge_recon = imfill(mask_edge_recon, 'holes');
        mask_edge_recon = mask_edge_recon & ~mask_chip_local;  % 再次排除
        
        % ── Step 4: 融合分水岭与边缘约束结果 ──
        % 取并集后再用强梯度线裁切
        mask_fused = mask_ws | mask_edge_recon;
        
        % 用自适应梯度阈值裁切：高于局部梯度均值+1.5倍标准差的视为边界
        G_vals = G_local_norm(mask_fused);
        if ~isempty(G_vals)
            G_mean = mean(G_vals);
            G_std = std(G_vals);
            G_adaptive = min(0.5, G_mean + 1.5 * G_std);
        else
            G_adaptive = 0.3;
        end
        mask_very_strong = G_local_norm > G_adaptive;
        
        % 去除粘连到强梯度线外侧的像素
        % 但保留被强梯度线包围的内部区域
        mask_interior = imfill(mask_very_strong, 'holes');
        mask_fused = mask_fused & (mask_interior | mask_fg_seed);
        
        % ── Step 5: 最终清理 ──
        % 强制排除芯片本体区域（最终保障）
        mask_fused = mask_fused & ~mask_chip_local;
        
        % 最终平滑
        mask_fused = imfill(mask_fused, 'holes');
        mask_fused = imclose(mask_fused, strel('disk', 1));
        mask_fused = imopen(mask_fused, strel('disk', 1));
        
        % 平滑后再次排除芯片区域（imclose/imfill可能重新渗透）
        mask_fused = mask_fused & ~mask_chip_local;
        
        % 确保结果不比粗掩膜差太多（防止过度裁切）
        % 如果精修后面积 < 粗掩膜的30%，回退到粗掩膜（仍排除芯片）
        new_area = sum(mask_fused(:));
        old_area = sum(mask_local(:));
        if new_area < old_area * 0.3
            mask_fused = mask_local & ~mask_chip_local;
            new_area = sum(mask_fused(:));
            fprintf('    引脚#%d 精修过度裁切，回退到粗掩膜\n', i);
        end
        
        % 写回全图
        mask_refined(ry1:ry2, rx1:rx2) = mask_refined(ry1:ry2, rx1:rx2) | mask_fused;
        
        % 更新引脚属性
        pins_refined(i).area = new_area;
        [rr, rc] = find(mask_fused);
        if ~isempty(rr)
            pins_refined(i).centroid = [mean(rc)+rx1-1, mean(rr)+ry1-1];
            pins_refined(i).bbox = [min(rc)+rx1-1, min(rr)+ry1-1, ...
                                    max(rc)-min(rc)+1, max(rr)-min(rr)+1];
        end
        
        fprintf('    引脚#%d 精修: %d → %d px\n', i, old_area, new_area);
    end
end
