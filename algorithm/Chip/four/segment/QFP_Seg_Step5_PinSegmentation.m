% fig5_10_5.m - 图5-10-5: 基于投影直方图的单个引脚分割与编号
% (a) 上侧引脚投影曲线
% (b) 间隙检测与分割位置
% (c) 分割后的单个引脚
% (d) 四侧引脚编号结果
%
% 输入：I_gray - 原始灰度图像
%       pins_regions - 各侧引脚区域
% 输出：pins_individual - 单个引脚信息结构体
%       pins_masks - 各引脚的二值掩膜
%
% 使用方法：运行测试脚本 test_fig5_10_5 或 test_qfp_all

function [pins_individual, pins_masks] = QFP_Seg_Step5_PinSegmentation(I_gray, pins_regions)
% 输入：I_gray - 原始灰度图像
%       pins_regions - 各侧引脚区域
% 输出：pins_individual - 单个引脚信息结构体
%       pins_masks - 各引脚的二值掩膜

sides = {'top', 'bottom', 'left', 'right'};
pins_individual = struct();
pins_masks = struct();

%% 对各侧分别进行投影分割
for s = 1:length(sides)
    side_name = sides{s};
    
    if ~isfield(pins_regions, side_name) || isempty(pins_regions.(side_name))
        pins_individual.(side_name) = [];
        pins_masks.(side_name) = [];
        continue;
    end
    
    fprintf('\n分割%s引脚...\n', side_name);
    
    % 构建该侧所有引脚的掩膜
    side_mask = false(size(I_gray));
    for i = 1:length(pins_regions.(side_name))
        side_mask(pins_regions.(side_name)(i).PixelIdxList) = true;
    end
    
    %% 投影分割
    if strcmp(side_name, 'top') || strcmp(side_name, 'bottom')
        % 水平排列 - 沿x轴投影
        projection = sum(side_mask, 1);  % 沿y方向求和
        axis_dir = 'x';
    else
        % 垂直排列 - 沿y轴投影
        projection = sum(side_mask, 2)';  % 沿x方向求和
        axis_dir = 'y';
    end
    
    % 检测间隙
    h_thr = max(projection) * 0.3;  % 阈值为最大值的30%
    gaps = find_gaps(projection, h_thr);
    
    % 基于间隙分割引脚
    if isempty(gaps)
        % 没有检测到间隙，整个区域作为一个引脚
        pins_individual.(side_name) = pins_regions.(side_name);
        pins_masks.(side_name) = {side_mask};
        fprintf('  未检测到间隙，保持整体\n');
    else
        % 根据间隙分割
        gap_centers = mean(gaps, 2);
        split_positions = [1, gap_centers', length(projection)];
        
        individual_pins = cell(length(split_positions)-1, 1);
        individual_masks = cell(length(split_positions)-1, 1);
        
        for i = 1:length(split_positions)-1
            pos_start = round(split_positions(i));
            pos_end = round(split_positions(i+1));
            
            % 提取该段引脚掩膜
            pin_mask = false(size(side_mask));
            if strcmp(axis_dir, 'x')
                pin_mask(:, pos_start:pos_end) = side_mask(:, pos_start:pos_end);
            else
                pin_mask(pos_start:pos_end, :) = side_mask(pos_start:pos_end, :);
            end
            
            % 确保连通
            CC = bwconncomp(pin_mask);
            if CC.NumObjects > 0
                % 选择最大连通域
                numPixels = cellfun(@numel, CC.PixelIdxList);
                [~, idx_max] = max(numPixels);
                pin_mask_connected = false(size(pin_mask));
                pin_mask_connected(CC.PixelIdxList{idx_max}) = true;
                
                individual_masks{i} = pin_mask_connected;
                individual_pins{i} = regionprops(pin_mask_connected, 'all');
            end
        end
        
        pins_individual.(side_name) = individual_pins;
        pins_masks.(side_name) = individual_masks;
        fprintf('  检测到 %d 个间隙，分割为 %d 个引脚\n', size(gaps,1), length(individual_pins));
    end
end

%% 对所有引脚进行编号（逆时针顺序：上侧从左到右，右侧从上到下，下侧从右到左，左侧从下到上）
pin_id = 1;
all_pins_info = [];

% 上侧：从左到右
if isfield(pins_individual, 'top') && ~isempty(pins_individual.top)
    n_top = length(pins_individual.top);
    for i = 1:n_top
        if ~isempty(pins_individual.top{i})
            info = struct();
            info.id = pin_id;
            info.side = 'top';
            info.centroid = pins_individual.top{i}.Centroid;
            info.mask = pins_masks.top{i};
            all_pins_info = [all_pins_info; info];
            pin_id = pin_id + 1;
        end
    end
end

% 右侧：从上到下（需要排序）
if isfield(pins_individual, 'right') && ~isempty(pins_individual.right)
    n_right = length(pins_individual.right);
    centroids_y = zeros(n_right, 1);
    for i = 1:n_right
        if ~isempty(pins_individual.right{i})
            centroids_y(i) = pins_individual.right{i}.Centroid(2);
        end
    end
    [~, sort_idx] = sort(centroids_y);
    for i = sort_idx'
        if ~isempty(pins_individual.right{i})
            info = struct();
            info.id = pin_id;
            info.side = 'right';
            info.centroid = pins_individual.right{i}.Centroid;
            info.mask = pins_masks.right{i};
            all_pins_info = [all_pins_info; info];
            pin_id = pin_id + 1;
        end
    end
end

% 下侧：从右到左（需要排序）
if isfield(pins_individual, 'bottom') && ~isempty(pins_individual.bottom)
    n_bottom = length(pins_individual.bottom);
    centroids_x = zeros(n_bottom, 1);
    for i = 1:n_bottom
        if ~isempty(pins_individual.bottom{i})
            centroids_x(i) = pins_individual.bottom{i}.Centroid(1);
        end
    end
    [~, sort_idx] = sort(centroids_x, 'descend');
    for i = sort_idx'
        if ~isempty(pins_individual.bottom{i})
            info = struct();
            info.id = pin_id;
            info.side = 'bottom';
            info.centroid = pins_individual.bottom{i}.Centroid;
            info.mask = pins_masks.bottom{i};
            all_pins_info = [all_pins_info; info];
            pin_id = pin_id + 1;
        end
    end
end

% 左侧：从下到上（需要排序）
if isfield(pins_individual, 'left') && ~isempty(pins_individual.left)
    n_left = length(pins_individual.left);
    centroids_y = zeros(n_left, 1);
    for i = 1:n_left
        if ~isempty(pins_individual.left{i})
            centroids_y(i) = pins_individual.left{i}.Centroid(2);
        end
    end
    [~, sort_idx] = sort(centroids_y, 'descend');
    for i = sort_idx'
        if ~isempty(pins_individual.left{i})
            info = struct();
            info.id = pin_id;
            info.side = 'left';
            info.centroid = pins_individual.left{i}.Centroid;
            info.mask = pins_masks.left{i};
            all_pins_info = [all_pins_info; info];
            pin_id = pin_id + 1;
        end
    end
end

pins_individual.all_pins = all_pins_info;

%% 可视化
figure('Position', [100, 100, 1400, 800], 'Color', 'w');

% (a) 上侧引脚投影曲线
subplot(2, 2, 1);
if isfield(pins_regions, 'top') && ~isempty(pins_regions.top)
    top_mask = false(size(I_gray));
    for i = 1:length(pins_regions.top)
        top_mask(pins_regions.top(i).PixelIdxList) = true;
    end
    projection_top = sum(top_mask, 1);
    
    yyaxis left
    plot(1:length(projection_top), projection_top, 'b-', 'LineWidth', 1.5);
    ylabel('投影计数', 'FontSize', 10);
    xlabel('X 坐标 (像素)', 'FontSize', 10);
    
    % 标注间隙
    h_thr = max(projection_top) * 0.3;
    hold on;
    yline(h_thr, 'r--', 'LineWidth', 1.5);
    gaps_top = find_gaps(projection_top, h_thr);
    for i = 1:size(gaps_top, 1)
        xline(mean(gaps_top(i,:)), 'g--', 'LineWidth', 1.5);
    end
    
    yyaxis right
    imagesc(1:length(projection_top), [0 1], repmat(projection_top > h_thr, [10 1]));
    ylabel('二值化', 'FontSize', 10);
end
title('(a) 上侧引脚投影曲线', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
legend('投影曲线', '阈值', '间隙位置', 'Location', 'best');
grid on;

% (b) 间隙检测与分割位置
subplot(2, 2, 2);
imshow(I_gray); hold on;
colors_side = [1 0 0; 0 1 0; 1 0 1; 0 1 1];
for s = 1:length(sides)
    side_name = sides{s};
    if isfield(pins_masks, side_name) && ~isempty(pins_masks.(side_name))
        for i = 1:length(pins_masks.(side_name))
            if ~isempty(pins_masks.(side_name){i})
                % 绘制每个引脚的轮廓
                boundary = bwboundaries(pins_masks.(side_name){i});
                if ~isempty(boundary)
                    plot(boundary{1}(:,2), boundary{1}(:,1), 'Color', colors_side(s,:), 'LineWidth', 2);
                end
            end
        end
    end
end
title('(b) 间隙检测与分割位置', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);

% (c) 分割后的单个引脚
subplot(2, 2, 3);
imshow(I_gray); hold on;
for i = 1:length(all_pins_info)
    mask_i = all_pins_info(i).mask;
    % 用不同颜色显示每个引脚
    color_i = hsv2rgb([mod(i*0.1, 1), 0.8, 0.9]);
    h = imshow(repmat(mask_i, [1 1 3]) .* reshape(color_i, [1 1 3]));
    set(h, 'AlphaData', mask_i * 0.6);
end
title(sprintf('(c) 分割后的单个引脚\n(共 %d 个)', length(all_pins_info)), ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);

% (d) 四侧引脚编号结果
subplot(2, 2, 4);
imshow(I_gray); hold on;
for i = 1:length(all_pins_info)
    centroid = all_pins_info(i).centroid;
    text(centroid(1), centroid(2), sprintf('%d', all_pins_info(i).id), ...
        'Color', 'y', 'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'k');
    
    % 绘制引脚轮廓
    boundary = bwboundaries(all_pins_info(i).mask);
    if ~isempty(boundary)
        plot(boundary{1}(:,2), boundary{1}(:,1), 'g-', 'LineWidth', 1);
    end
end
title('(d) 四侧引脚编号结果', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);

sgtitle('图5-10-5 基于投影直方图的单个引脚分割与编号', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

fprintf('\n引脚分割与编号完成，共 %d 个引脚\n', length(all_pins_info));

end

%% 辅助函数：检测投影曲线中的间隙
function gaps = find_gaps(projection, threshold)
% projection: 投影曲线
% threshold: 间隙阈值

below_thr = projection < threshold;
diff_below = diff([0, below_thr, 0]);

gap_starts = find(diff_below == 1);
gap_ends = find(diff_below == -1) - 1;

% 过滤太窄的间隙（宽度<3像素）
gap_widths = gap_ends - gap_starts + 1;
valid_gaps = gap_widths >= 3;

gaps = [gap_starts(valid_gaps)', gap_ends(valid_gaps)'];

end
