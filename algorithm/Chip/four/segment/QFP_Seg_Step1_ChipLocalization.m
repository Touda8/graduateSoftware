% fig5_10_1.m - 图5-10: 扁平封装芯片区域定位流程与结果
%
% 功能：实现扁平封装芯片区域的粗定位与精定位
%
% 解决问题：
%   1. 芯片体与周围阴影灰度接近，单一大津阈值无法分离
%   2. 图像边缘暗角与芯片区域灰度相近产生粘连
%   3. 粗定位获得的掩膜边界精度仅为像素级，需精确到亚像素
%
% 核心方法：
%   粗定位阶段：分层大津阈值 (Hierarchical Otsu)
%     第1级：全局大津 → 分出暗区(芯片+阴影) vs 亮区(背景+引脚)
%     第2级：仅在暗区像素上再做大津 → 分出芯片体(极暗) vs 阴影(中暗)
%     再用 imclearborder 移除边缘暗角，形状约束筛选芯片体
%   精定位阶段：由内向外灰度跳变扫描 + 四边直线拟合
%     从粗掩膜质心出发，沿四个方向向外扫描，找灰度首次跳变位置
%     在跳变点附近用局部梯度最大值做亚像素精化
%     用 IRLS 拟合四条直线边界，四线交点构成精确矩形掩膜
%
% 输入：I_original - 原始灰度图像或图像路径
% 输出：I_chip - 芯片区域图像
%       mask_chip - 芯片区域掩膜（精确矩形）
%       mask_chip_coarse - 粗定位掩膜（用于中间可视化）
%       chip_info - 芯片几何信息结构体（含四条IRLS直线、角点等）
%         .corners    - 4×2 角点坐标 [x,y]（左上/右上/右下/左下）
%         .line_top   - 上边直线 [a,b,c] (ax+by+c=0)
%         .line_bot   - 下边直线
%         .line_left  - 左边直线
%         .line_right - 右边直线
%
% 使用方法：
%   [I_chip, mask_chip, mask_chip_coarse, chip_info] = QFP_Seg_Step1_ChipLocalization('fourchip.png');
%   或运行测试脚本：test_fig5_10_1

function [I_chip, mask_chip, mask_chip_coarse, chip_info] = QFP_Seg_Step1_ChipLocalization(I_original)

%% 1. 读取原始图像
if ischar(I_original)
    I_original = imread(I_original);
end
if size(I_original, 3) == 3
    I_gray = rgb2gray(I_original);
else
    I_gray = I_original;
end
I_gray = im2double(I_gray);
[img_h, img_w] = size(I_gray);
img_area = img_h * img_w;

%% ====================================================================
%%  Phase 1: 分层大津阈值粗定位 (保留原有方法)
%% ====================================================================

%% 2. 分层大津阈值 (Hierarchical Otsu)
%  单级大津 T1≈0.56 把芯片体+阴影+暗角全部归为"暗"，无法分离。
%  分层策略：
%    Level-1: 全局大津 T1 → 取暗区像素 (I < T1)
%    Level-2: 仅在暗区上再做大津 → T2 自适应切在芯片体和阴影之间。

% Level-1: 全局大津
T1 = graythresh(I_gray);
mask_dark = I_gray <= T1;     % 所有暗区像素
I_binary_otsu = ~imbinarize(I_gray, T1);  % 用于可视化对比

% Level-2: 暗区内部再做大津
dark_vals = I_gray(mask_dark);
T2 = graythresh(dark_vals);   % 暗区内部的最优阈值
mask_chipbody = I_gray <= T2; % 芯片体 = 极暗像素

fprintf('分层大津阈值: T1(全局)=%.4f, T2(暗区内)=%.4f\n', T1, T2);
fprintf('  T2/T1 = %.2f（芯片体阈值仅为全局阈值的 %.0f%%）\n', T2/T1, T2/T1*100);

% 三类可视化图
I_3class = ones(size(I_gray));           % 白 = 亮区(背景)
I_3class(mask_dark) = 0.5;              % 灰 = 阴影
I_3class(mask_chipbody) = 0;            % 黑 = 芯片体

%% 3. 移除边缘暗角 (imclearborder)
% Step 3a: 腐蚀断开芯片体与边缘暗角之间可能的薄弱连接
se_break = strel('disk', 3);
mask_eroded = imerode(mask_chipbody, se_break);

% Step 3b: 清除接触边界的连通域
mask_interior = imclearborder(mask_eroded);

% Step 3c: 检查是否足够（防止芯片也被清除的极端情况）
interior_ratio = sum(mask_interior(:)) / img_area;
fprintf('  imclearborder 后残留: %.2f%% 图像面积\n', interior_ratio*100);

if interior_ratio < 0.003
    fprintf('  警告: imclearborder 消除过多，启用边界距离回退\n');
    [XX, YY] = meshgrid(1:img_w, 1:img_h);
    dist_border = min(cat(3, XX, YY, img_w+1-XX, img_h+1-YY), [], 3);
    margin = min(img_h, img_w) * 0.08;
    mask_interior = mask_chipbody & (dist_border > margin);
end

%% 4. 形态学精修 ——恢复腐蚀 + 填充内部
% 膨胀恢复被腐蚀掉的边界（适度膨胀，避免超出实际边缘）
se_recover = strel('disk', 4);
mask_recovered = imdilate(mask_interior, se_recover);

% 闭运算弥合芯片表面印刷文字导致的缝隙
se_close = strel('disk', 10);
mask_clean = imclose(mask_recovered, se_close);

% 填充芯片体内部的孔洞（印刷标记/反光点）
mask_clean = imfill(mask_clean, 'holes');

%% 5. 连通域分析 ——形状约束筛选芯片体
CC = bwconncomp(mask_clean);
numPixels = cellfun(@numel, CC.PixelIdxList);
img_center = [img_w/2, img_h/2];

% 面积约束：芯片体应占图像面积 0.5%~50%
area_lo = 0.005 * img_area;
area_hi = 0.50  * img_area;
valid_idx = find(numPixels > area_lo & numPixels < area_hi);

if isempty(valid_idx)
    valid_idx = find(numPixels > 0.002 * img_area);
    if isempty(valid_idx)
        warning('未找到芯片候选，使用最大连通域');
        [~, valid_idx] = max(numPixels);
    end
end

% 综合评分选择最佳候选
best_score = -inf;
idx_best = valid_idx(1);
for i = 1:length(valid_idx)
    idx = valid_idx(i);
    temp = false(img_h, img_w);
    temp(CC.PixelIdxList{idx}) = true;
    s = regionprops(temp, 'Area', 'BoundingBox', 'Centroid', ...
                    'Solidity', 'Extent');
    
    ar = s.BoundingBox(4) / max(s.BoundingBox(3), 1);
    if ar > 1, ar = 1/ar; end
    
    dist_norm = norm(s.Centroid - img_center) / norm([img_w, img_h]);
    
    score = s.Extent*0.30 + s.Solidity*0.25 + ar*0.25 - dist_norm*0.20;
    
    fprintf('  候选%d: 面积=%d, 矩形度=%.2f, 紧凑度=%.2f, 中心距=%.2f, 得分=%.3f\n', ...
            i, s.Area, s.Extent, s.Solidity, dist_norm, score);
    
    if score > best_score
        best_score = score;
        idx_best = idx;
    end
end

mask_chip_raw = false(img_h, img_w);
mask_chip_raw(CC.PixelIdxList{idx_best}) = true;

%% ====================================================================
%%  Phase 2: 从芯片内部向外扫描，精确定位真实边缘
%% ====================================================================
%  ── 从结果图分析的问题 ──
%  问题1: 搜索带内"最大梯度"策略失效：引脚边缘梯度比芯片体边缘更强，
%         导致梯度极值点落在引脚外侧而非芯片体边缘。
%  问题2: 排除两端15%不够——引脚根部就在芯片体边缘处，梯度仍被引脚主导。
%  问题3: 粗掩膜BoundingBox已被形态学膨胀扩大，搜索带定位偏外。
%
%  ── 正确策略：由内向外扫描 ──
%  芯片体内部极暗(灰度<T2≈0.27)，边缘外侧显著变亮。
%  从粗掩膜的**内部中心**出发，沿四个方向向外扫描每一行/列的灰度，
%  找到灰度从暗到亮的**第一个显著跳变位置**，即真实的芯片体边缘。
%  这样天然地避开引脚干扰——引脚在芯片体外侧，扫描时已经越过了真实边缘。
%
%  具体实现：
%  (1) 找粗掩膜质心，确定芯片体内部基准区域
%  (2) 上/下边：对中间60%的列，从质心行向上/向下扫描，
%      找灰度首次超过 T2 的位置（暗→亮跳变点）
%  (3) 左/右边：对中间60%的行，从质心列向左/向右扫描
%  (4) IRLS 拟合四条直线，自动剔除异常点

fprintf('\n===== Phase 2: 由内向外扫描边缘精修 =====\n');

%% 6. 利用粗掩膜确定芯片内部基准
s_coarse = regionprops(mask_chip_raw, 'BoundingBox', 'Centroid');
bbox = s_coarse.BoundingBox;  % [x, y, w, h]
chip_cx = round(s_coarse.Centroid(1));
chip_cy = round(s_coarse.Centroid(2));
chip_w = bbox(3);
chip_h = bbox(4);

x_lo = round(bbox(1));
y_lo = round(bbox(2));
x_hi = round(bbox(1) + bbox(3));
y_hi = round(bbox(2) + bbox(4));

fprintf('  粗掩膜中心: (%d, %d), 尺寸: %.0f×%.0f\n', chip_cx, chip_cy, chip_w, chip_h);

%% 7. 计算梯度（仅用于可视化和辅助验证）
I_smooth = imgaussfilt(I_gray, 1.5);
[Gx, Gy] = imgradientxy(I_smooth, 'sobel');
G_mag = sqrt(Gx.^2 + Gy.^2);

%% 8. 确定灰度跳变阈值
%  芯片体内部灰度 < T2，边缘外侧灰度显著高于 T2。
%  使用 T2 和一个稍高的阈值(T_edge)作为跳变检测门限。
%  T_edge 取 T2 与 T1 的加权平均，确保跳过芯片表面纹理的微小灰度波动。
T_edge = T2 + (T1 - T2) * 0.3;  % 偏靠 T2，敏感于边缘跳变
fprintf('  边缘跳变阈值: T_edge=%.4f (T2=%.4f, T1=%.4f)\n', T_edge, T2, T1);

% 搜索范围限制：从质心出发，最远不超过粗掩膜外扩 20 像素
search_margin = 20;

%% 9. 由内向外扫描四条边
%  ---- 上边界：中间60%的列，每列从质心行向上扫描 ----
fprintf('  扫描上边界...\n');
x_trim = round(chip_w * 0.20);  % 排除两端各20%（引脚区域更宽裕）
x_cols = max(1, x_lo + x_trim) : min(img_w, x_hi - x_trim);
y_limit_top = max(1, y_lo - search_margin);

pts_top = zeros(length(x_cols), 2);
cnt_top = 0;
for i = 1:length(x_cols)
    col = x_cols(i);
    % 从质心行向上扫描（y递减）
    for row = chip_cy : -1 : y_limit_top
        if I_gray(row, col) > T_edge
            % 找到第一个灰度超过阈值的位置 = 已经出了芯片体
            % 真实边缘在这个位置和上一个暗像素之间
            % 在跳变点附近做亚像素精化：找局部梯度最大值
            search_lo = max(y_limit_top, row - 5);
            search_hi = min(chip_cy, row + 5);
            g_local = G_mag(search_lo:search_hi, col);
            [~, local_max_idx] = max(g_local);
            edge_y = search_lo + local_max_idx - 1;
            
            cnt_top = cnt_top + 1;
            pts_top(cnt_top, :) = [col, edge_y];
            break;
        end
    end
end
pts_top = pts_top(1:cnt_top, :);
fprintf('    提取到 %d 个边缘点\n', cnt_top);

%  ---- 下边界：中间60%的列，每列从质心行向下扫描 ----
fprintf('  扫描下边界...\n');
y_limit_bot = min(img_h, y_hi + search_margin);

pts_bot = zeros(length(x_cols), 2);
cnt_bot = 0;
for i = 1:length(x_cols)
    col = x_cols(i);
    for row = chip_cy : 1 : y_limit_bot
        if I_gray(row, col) > T_edge
            search_lo = max(chip_cy, row - 5);
            search_hi = min(y_limit_bot, row + 5);
            g_local = G_mag(search_lo:search_hi, col);
            [~, local_max_idx] = max(g_local);
            edge_y = search_lo + local_max_idx - 1;
            
            cnt_bot = cnt_bot + 1;
            pts_bot(cnt_bot, :) = [col, edge_y];
            break;
        end
    end
end
pts_bot = pts_bot(1:cnt_bot, :);
fprintf('    提取到 %d 个边缘点\n', cnt_bot);

%  ---- 左边界：中间60%的行，每行从质心列向左扫描 ----
fprintf('  扫描左边界...\n');
y_trim = round(chip_h * 0.20);
y_rows = max(1, y_lo + y_trim) : min(img_h, y_hi - y_trim);
x_limit_left = max(1, x_lo - search_margin);

pts_left = zeros(length(y_rows), 2);
cnt_left = 0;
for i = 1:length(y_rows)
    row = y_rows(i);
    for col = chip_cx : -1 : x_limit_left
        if I_gray(row, col) > T_edge
            search_lo = max(x_limit_left, col - 5);
            search_hi = min(chip_cx, col + 5);
            g_local = G_mag(row, search_lo:search_hi);
            [~, local_max_idx] = max(g_local);
            edge_x = search_lo + local_max_idx - 1;
            
            cnt_left = cnt_left + 1;
            pts_left(cnt_left, :) = [edge_x, row];
            break;
        end
    end
end
pts_left = pts_left(1:cnt_left, :);
fprintf('    提取到 %d 个边缘点\n', cnt_left);

%  ---- 右边界：中间60%的行，每行从质心列向右扫描 ----
fprintf('  扫描右边界...\n');
x_limit_right = min(img_w, x_hi + search_margin);

pts_right = zeros(length(y_rows), 2);
cnt_right = 0;
for i = 1:length(y_rows)
    row = y_rows(i);
    for col = chip_cx : 1 : x_limit_right
        if I_gray(row, col) > T_edge
            search_lo = max(chip_cx, col - 5);
            search_hi = min(x_limit_right, col + 5);
            g_local = G_mag(row, search_lo:search_hi);
            [~, local_max_idx] = max(g_local);
            edge_x = search_lo + local_max_idx - 1;
            
            cnt_right = cnt_right + 1;
            pts_right(cnt_right, :) = [edge_x, row];
            break;
        end
    end
end
pts_right = pts_right(1:cnt_right, :);
fprintf('    提取到 %d 个边缘点\n', cnt_right);

%% 10. IRLS 直线拟合（剔除异常点）
fprintf('  IRLS 拟合上边界...\n');
[line_top, inlier_top] = irls_line_fit(pts_top, 'horizontal', 8, 1.5);
fprintf('  IRLS 拟合下边界...\n');
[line_bot, inlier_bot] = irls_line_fit(pts_bot, 'horizontal', 8, 1.5);
fprintf('  IRLS 拟合左边界...\n');
[line_left, inlier_left] = irls_line_fit(pts_left, 'vertical', 8, 1.5);
fprintf('  IRLS 拟合右边界...\n');
[line_right, inlier_right] = irls_line_fit(pts_right, 'vertical', 8, 1.5);

%% 11. 计算四线交点，构成精确矩形
corners = zeros(4, 2);  % [x, y] 四个角点
corners(1,:) = line_intersection(line_top, line_left);    % 左上
corners(2,:) = line_intersection(line_top, line_right);   % 右上
corners(3,:) = line_intersection(line_bot, line_right);   % 右下
corners(4,:) = line_intersection(line_bot, line_left);    % 左下

fprintf('  精确矩形角点:\n');
corner_names = {'左上', '右上', '右下', '左下'};
for i = 1:4
    fprintf('    %s: (%.1f, %.1f)\n', corner_names{i}, corners(i,1), corners(i,2));
end

% 验证矩形合理性
corners_valid = all(corners(:,1) > 0 & corners(:,1) <= img_w & ...
                    corners(:,2) > 0 & corners(:,2) <= img_h);

% 拟合矩形的面积应与粗掩膜面积相近（0.5x~1.5x）
coarse_area = sum(mask_chip_raw(:));
poly_area = polyarea(corners(:,1), corners(:,2));
area_ratio = poly_area / coarse_area;
fprintf('  拟合面积/粗掩膜面积 = %.2f\n', area_ratio);
corners_reasonable = area_ratio > 0.5 && area_ratio < 1.5;

if ~corners_valid || ~corners_reasonable
    fprintf('  警告: 直线拟合结果异常(valid=%d, ratio=%.2f)，回退到凸包方案\n', ...
            corners_valid, area_ratio);
    mask_rect = bwconvhull(mask_chip_raw);
else
    %% 12. 用四角点生成精确矩形掩膜
    mask_rect = poly2mask(corners(:,1), corners(:,2), img_h, img_w);
    fprintf('  精确矩形掩膜生成完成\n');
end

% 保存粗定位掩膜供输出使用
mask_chip_coarse = mask_chip_raw;

%% ====================================================================
%%  Phase 3: 矩形内部像素级精确分割
%% ====================================================================
%  ── 从结果图分析的问题 ──
%  (g)图：绿色像素级边界几乎贴着矩形走，没有真正贴合芯片体边缘。
%  原因：矩形掩膜是从四线交点构成的，覆盖了部分芯片体外的阴影/背景。
%
%  ── 正确策略 ──
%  直接复用 Phase 1 已精确计算的 T2 阈值（芯片体vs阴影分界线），
%  在矩形ROI内做阈值分割 + 多尺度形态学精修：
%  (1) 用全局 T2 阈值在ROI内分割（T2 专为分离芯片体而设计）
%  (2) 适度放宽阈值到 T2*1.15 捕获边缘渐变带的最后暗像素
%  (3) 用梯度加权的自适应膨胀：在强梯度(真实边缘)处停止膨胀，
%      在弱梯度(芯片表面内部)处继续填充
%  (4) 连通域约束 + 与矩形取交集

fprintf('\n===== Phase 3: 矩形内部像素级精确分割 =====\n');

% 矩形外扩余量
expand_px = 5;
roi_x1 = max(1, round(min(corners(:,1))) - expand_px);
roi_y1 = max(1, round(min(corners(:,2))) - expand_px);
roi_x2 = min(img_w, round(max(corners(:,1))) + expand_px);
roi_y2 = min(img_h, round(max(corners(:,2))) + expand_px);

I_roi = I_gray(roi_y1:roi_y2, roi_x1:roi_x2);
[roi_h, roi_w] = size(I_roi);
fprintf('  ROI范围: (%d,%d)-(%d,%d), 尺寸: %d×%d\n', ...
        roi_x1, roi_y1, roi_x2, roi_y2, roi_w, roi_h);

% ── Step 1: 用全局 T2 阈值做严格分割 ──
% T2 是 Phase 1 中在暗区上二次大津得到的，精确分离芯片体(极暗)与阴影(中暗)
% 适度放宽 15%，捕获芯片体边缘渐变带的最后一层暗像素
T_strict = T2 * 1.15;
mask_roi_strict = I_roi <= T_strict;
fprintf('  严格阈值: T_strict=%.4f (T2×1.15)\n', T_strict);

% ── Step 2: 形态学清理 ──
% 闭运算填充芯片表面印刷文字的缝隙
se_close_roi = strel('disk', 4);
mask_roi_closed = imclose(mask_roi_strict, se_close_roi);

% 填充内部孔洞（印字/反光点）
mask_roi_filled = imfill(mask_roi_closed, 'holes');

% ── Step 3: 提取最大连通域 ──
CC_roi = bwconncomp(mask_roi_filled);
if CC_roi.NumObjects > 0
    numPixels_roi = cellfun(@numel, CC_roi.PixelIdxList);
    [~, idx_max_roi] = max(numPixels_roi);
    mask_roi_main = false(roi_h, roi_w);
    mask_roi_main(CC_roi.PixelIdxList{idx_max_roi}) = true;
else
    mask_roi_main = mask_roi_filled;
end

% ── Step 4: 梯度引导的边缘精修 ──
% 问题：T2阈值分割的边缘可能有1-3像素的锯齿（因为灰度渐变带）
% 解决：用形态学膨胀恢复可能被阈值切掉的边缘暗像素，
%       但在高梯度位置（真实边缘）停止膨胀。

% 计算ROI内的梯度
I_roi_smooth = imgaussfilt(I_roi, 1.0);
G_roi = imgradient(I_roi_smooth, 'sobel');

% 归一化梯度到 [0, 1]
G_roi_norm = G_roi / max(G_roi(:) + eps);

% 创建梯度约束膨胀掩膜：梯度 < 阈值的区域允许膨胀
% 高梯度 = 真实边缘 → 不允许越过
G_threshold = 0.25;  % 归一化梯度阈值
mask_low_gradient = G_roi_norm < G_threshold;

% 条件膨胀：最多膨胀 2 像素，且只膨胀到低梯度区域
% 同时要求目标像素的灰度仍然足够暗（<= T1，即全局大津的暗区范围）
mask_expandable = mask_low_gradient & (I_roi <= T1);
se_expand = strel('disk', 2);
mask_roi_dilated = imdilate(mask_roi_main, se_expand);
mask_roi_refined = mask_roi_main | (mask_roi_dilated & mask_expandable);

% 再次填充可能出现的孔洞
mask_roi_refined = imfill(mask_roi_refined, 'holes');

% ── Step 5: 与矩形取交集 ──
mask_rect_roi = mask_rect(roi_y1:roi_y2, roi_x1:roi_x2);
mask_roi_final = mask_roi_refined & mask_rect_roi;

% ── Step 6: 最终连通域清理 ──
CC_final = bwconncomp(mask_roi_final);
if CC_final.NumObjects > 1
    numPixels_final = cellfun(@numel, CC_final.PixelIdxList);
    [~, idx_max_final] = max(numPixels_final);
    mask_roi_result = false(roi_h, roi_w);
    mask_roi_result(CC_final.PixelIdxList{idx_max_final}) = true;
else
    mask_roi_result = mask_roi_final;
end

% ── Step 7: 轻微平滑锯齿 ──
se_smooth = strel('disk', 1);
mask_roi_result = imclose(mask_roi_result, se_smooth);
mask_roi_result = imopen(mask_roi_result, se_smooth);

% ── Step 8: 写回全图 ──
mask_chip = false(img_h, img_w);
mask_chip(roi_y1:roi_y2, roi_x1:roi_x2) = mask_roi_result;

% 统计
pixel_diff = sum(mask_rect(:)) - sum(mask_chip(:));
fprintf('  像素级精修完成: 去除了 %d 个非芯片像素 (%.1f%%)\n', ...
        pixel_diff, 100*pixel_diff/sum(mask_rect(:)));

s_pixel = regionprops(mask_chip, 'Solidity', 'Extent', 'Area');
fprintf('  精修后: 面积=%d, 矩形度=%.3f, 紧凑度=%.3f\n', ...
        s_pixel.Area, s_pixel.Extent, s_pixel.Solidity);

%% 13. 构造芯片几何信息（供 fig5_10_3 使用）
chip_info.corners    = corners;      % 4×2 [x,y]
chip_info.line_top   = line_top;     % [a,b,c]
chip_info.line_bot   = line_bot;
chip_info.line_left  = line_left;
chip_info.line_right = line_right;

%% 14. 提取芯片区域图像
I_chip = I_gray .* double(mask_chip);

%% ====================================================================
%%  图5-10-1: 粗定位流程可视化（2×3布局）
%% ====================================================================
figure('Name', '图5-10-1 芯片区域粗定位流程', ...
       'Position', [50, 100, 1400, 700], 'Color', 'w');

%% ====================================================================
%%  图5-10-1: 粗定位流程可视化（2×3布局）
%% ====================================================================
figure('Name', '图5-10-1 芯片区域粗定位流程', ...
       'Position', [50, 100, 1400, 700], 'Color', 'w');

% (a) 原始灰度图像
subplot(2,3,1);
imshow(I_gray);
% title('(a) 原始灰度图像', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (b) 单级大津（对比失败案例）
subplot(2,3,2);
imshow(I_binary_otsu);
% title(sprintf('(b) 单级大津分割\nT=%.3f（粘连失败）', T1), ...
%       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (c) 分层大津三类分割
subplot(2,3,3);
imshow(I_3class);
% title(sprintf('(c) 分层大津三类分割\nT_1=%.3f, T_2=%.3f', T1, T2), ...
%       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (d) 极暗类提取（芯片体候选）
subplot(2,3,4);
imshow(mask_chipbody);
% title(sprintf('(d) 极暗类提取\nI<%.3f', T2), ...
%       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (e) 形态学精修+形状筛选
subplot(2,3,5);
imshow(mask_chip_raw);
s_raw = regionprops(mask_chip_raw, 'Area', 'Extent', 'Solidity');
% if ~isempty(s_raw)
%     title(sprintf('(e) 形态学+形状筛选\n面积=%d, 矩形度=%.2f', ...
%           s_raw.Area, s_raw.Extent), ...
%           'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
% end

% (f) 粗定位结果叠加到原图
subplot(2,3,6);
imshow(I_gray); hold on;
bnd_coarse = bwboundaries(mask_chip_raw);
for k = 1:length(bnd_coarse)
    plot(bnd_coarse{k}(:,2), bnd_coarse{k}(:,1), 'y-', 'LineWidth', 2);
end
hold off;
% title('(f) 粗定位边界叠加', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% sgtitle('图5-10-1 扁平封装芯片区域粗定位流程（分层大津阈值）', ...
%         'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

fprintf('\n粗定位完成：T1=%.4f, T2=%.4f, 面积=%d像素 (%.1f%%)\n', ...
        T1, T2, s_raw.Area, 100*s_raw.Area/img_area);

%% ==============================================================
%%  图5-10-2: 精定位流程（2×3布局）
%% ==============================================================
figure('Name', '图5-10-2 芯片区域精定位流程', ...
       'Position', [100, 50, 1400, 700], 'Color', 'w');

% (a) 粗定位掩膜与质心
subplot(2,3,1);
imshow(I_gray); hold on;
% 绘制粗掩膜边界
for k = 1:length(bnd_coarse)
    plot(bnd_coarse{k}(:,2), bnd_coarse{k}(:,1), 'y--', 'LineWidth', 1.5);
end
% 绘制质心和BoundingBox
plot(chip_cx, chip_cy, 'r+', 'MarkerSize', 15, 'LineWidth', 2);
rectangle('Position', bbox, 'EdgeColor', 'c', 'LineWidth', 1.5, 'LineStyle', '--');
hold off;
% title('(a) 粗定位掩膜与质心', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (b) 由内向外扫描示意
subplot(2,3,2);
imshow(I_gray); hold on;
% 绘制质心
plot(chip_cx, chip_cy, 'r+', 'MarkerSize', 15, 'LineWidth', 2);
% 绘制扫描列范围（上下边）
plot([x_cols(1), x_cols(end)], [chip_cy, chip_cy], 'g-', 'LineWidth', 1.5);
% 绘制扫描行范围（左右边）
plot([chip_cx, chip_cx], [y_rows(1), y_rows(end)], 'c-', 'LineWidth', 1.5);
% 绘制四个方向的扫描箭头
arrow_len = 30;
quiver(chip_cx, chip_cy, 0, -arrow_len, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);
quiver(chip_cx, chip_cy, 0, arrow_len, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);
quiver(chip_cx, chip_cy, -arrow_len, 0, 0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);
quiver(chip_cx, chip_cy, arrow_len, 0, 0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);
hold off;
% title(sprintf('(b) 由内向外扫描\nT_{edge}=%.3f', T_edge), ...
%       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (c) 四边边缘点提取
subplot(2,3,3);
imshow(I_gray); hold on;
% 绘制提取到的边缘点
if cnt_top > 0, plot(pts_top(:,1), pts_top(:,2), 'r.', 'MarkerSize', 4); end
if cnt_bot > 0, plot(pts_bot(:,1), pts_bot(:,2), 'r.', 'MarkerSize', 4); end
if cnt_left > 0, plot(pts_left(:,1), pts_left(:,2), 'b.', 'MarkerSize', 4); end
if cnt_right > 0, plot(pts_right(:,1), pts_right(:,2), 'b.', 'MarkerSize', 4); end
hold off;
% title(sprintf('(c) 边缘点提取\n上%d 下%d 左%d 右%d个', ...
%       cnt_top, cnt_bot, cnt_left, cnt_right), ...
%       'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (d) IRLS直线拟合结果
subplot(2,3,4);
imshow(I_gray); hold on;
% 绘制拟合直线
draw_line_on_image(line_top, img_w, img_h, 'r', 2);
draw_line_on_image(line_bot, img_w, img_h, 'r', 2);
draw_line_on_image(line_left, img_w, img_h, 'b', 2);
draw_line_on_image(line_right, img_w, img_h, 'b', 2);
% 绘制角点
if exist('corners', 'var') && corners_valid
    plot(corners(:,1), corners(:,2), 'yo', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'y');
end
hold off;
% title('(d) IRLS四边直线拟合', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (e) 矩形掩膜
subplot(2,3,5);
imshow(mask_rect);
s_rect = regionprops(mask_rect, 'Area', 'Extent', 'Solidity');
% title(sprintf('(e) 矩形掩膜\n面积=%d, 矩形度=%.2f', ...
%       s_rect.Area, s_rect.Extent), ...
%       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');

% (f) 矩形 vs 像素级精修对比
subplot(2,3,6);
imshow(I_gray); hold on;
% 矩形边界（红色虚线）
bnd_rect = bwboundaries(mask_rect);
for k = 1:length(bnd_rect)
    plot(bnd_rect{k}(:,2), bnd_rect{k}(:,1), 'r--', 'LineWidth', 1.5);
end
% 像素级精修边界（绿色实线）
bnd_fine = bwboundaries(mask_chip);
for k = 1:length(bnd_fine)
    plot(bnd_fine{k}(:,2), bnd_fine{k}(:,1), 'g-', 'LineWidth', 2);
end
hold off;
% title({'(f) 矩形 vs 像素级精修', '红=矩形, 绿=精修'}, ...
%       'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimHei');
% 
% sgtitle('图5-10-2 扁平封装芯片区域精定位流程（由内向外扫描+四线拟合+像素级精修）', ...
%         'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

fprintf('\n精定位完成：\n');
if exist('corners', 'var') && corners_valid
    fprintf('  矩形角点: 左上(%.1f,%.1f) 右上(%.1f,%.1f) 右下(%.1f,%.1f) 左下(%.1f,%.1f)\n', ...
            corners(1,1), corners(1,2), corners(2,1), corners(2,2), ...
            corners(3,1), corners(3,2), corners(4,1), corners(4,2));
end
s_final = regionprops(mask_chip, 'Area', 'Extent', 'Solidity');
fprintf('  矩形掩膜面积: %d像素\n', s_rect.Area);
fprintf('  像素级精修面积: %d像素 (%.1f%%)\n', s_final.Area, 100*s_final.Area/img_area);
fprintf('  最终矩形度: %.3f, 紧凑度: %.3f\n', s_final.Extent, s_final.Solidity);
fprintf('  最终矩形度: %.3f, 紧凑度: %.3f\n', s_final.Extent, s_final.Solidity);

end

%% ====================================================================
%%  辅助函数
%% ====================================================================

function [line_params, inlier_mask] = irls_line_fit(pts, orientation, max_iter, sigma_k)
% 迭代加权最小二乘 (IRLS) 直线拟合
% 输入：pts = [x, y]
%       orientation = 'horizontal'(y=kx+b) 或 'vertical'(x=ky+b)
%       max_iter = 最大迭代次数
%       sigma_k = Huber权重的sigma倍数
% 输出：line_params = [a, b, c] 使得 ax + by + c = 0
%       inlier_mask = 内点逻辑索引

    n = size(pts, 1);
    if n < 3
        line_params = [0, 1, 0];
        inlier_mask = true(n, 1);
        return;
    end
    
    if strcmp(orientation, 'horizontal')
        u = pts(:, 1);  % x
        v = pts(:, 2);  % y → y = ku + b
    else
        u = pts(:, 2);  % y
        v = pts(:, 1);  % x → x = ky + b
    end
    
    w = ones(n, 1);
    
    for iter = 1:max_iter
        % 加权最小二乘
        W = diag(w);
        M = [u, ones(n, 1)];
        p = (M' * W * M) \ (M' * W * v);
        k = p(1);
        b = p(2);
        
        % 计算残差
        residuals = abs(v - k*u - b);
        
        % 用MAD(中位数绝对偏差)估计稳健标准差
        sigma = 1.4826 * median(residuals);
        sigma = max(sigma, 0.5);  % 防止sigma过小
        
        % Huber 权重
        threshold = sigma_k * sigma;
        w = ones(n, 1);
        outlier = residuals > threshold;
        w(outlier) = threshold ./ residuals(outlier);
        
        fprintf('    IRLS iter %d: k=%.4f, b=%.1f, sigma=%.2f, 离群=%d/%d\n', ...
                iter, k, b, sigma, sum(outlier), n);
    end
    
    % 最终内点判定
    inlier_mask = residuals <= sigma_k * sigma;
    
    % 转换为一般式 ax + by + c = 0
    if strcmp(orientation, 'horizontal')
        % y = kx + b → kx - y + b = 0
        line_params = [k, -1, b];
    else
        % x = ky + b → -x + ky + b = 0 → x - ky - b = 0
        line_params = [1, -k, -b];
    end
    
    fprintf('    最终: %d/%d 内点\n', sum(inlier_mask), n);
end

function pt = line_intersection(L1, L2)
% 求两直线 a1*x+b1*y+c1=0 和 a2*x+b2*y+c2=0 的交点
    A = [L1(1), L1(2); L2(1), L2(2)];
    b = [-L1(3); -L2(3)];
    pt = (A \ b)';
end

function draw_line_on_image(L, img_w, img_h, color, lw)
% 在图像范围内绘制直线 ax+by+c=0
    a = L(1); b = L(2); c = L(3);
    pts = [];
    
    if abs(b) > 1e-10
        % y = (-ax - c) / b
        y_at_x1 = (-a*1 - c) / b;
        y_at_xw = (-a*img_w - c) / b;
        if y_at_x1 >= 1 && y_at_x1 <= img_h
            pts = [pts; 1, y_at_x1];
        end
        if y_at_xw >= 1 && y_at_xw <= img_h
            pts = [pts; img_w, y_at_xw];
        end
    end
    
    if abs(a) > 1e-10
        % x = (-by - c) / a
        x_at_y1 = (-b*1 - c) / a;
        x_at_yh = (-b*img_h - c) / a;
        if x_at_y1 >= 1 && x_at_y1 <= img_w
            pts = [pts; x_at_y1, 1];
        end
        if x_at_yh >= 1 && x_at_yh <= img_w
            pts = [pts; x_at_yh, img_h];
        end
    end
    
    if size(pts, 1) >= 2
        % 取距离最远的两个交点
        dists = pdist2(pts, pts);
        [~, idx] = max(dists(:));
        [i1, i2] = ind2sub(size(dists), idx);
        plot([pts(i1,1), pts(i2,1)], [pts(i1,2), pts(i2,2)], ...
             [color, '-'], 'LineWidth', lw);
    end
end


