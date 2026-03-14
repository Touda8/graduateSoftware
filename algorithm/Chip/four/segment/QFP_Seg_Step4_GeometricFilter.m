% fig5_10_4.m - 图5-10-4: 基于几何约束的四侧引脚区域筛选
% (a) 局部阈值分割结果
% (b) 长宽比与面积筛选
% (c) 方向角约束筛选
% (d) 位置约束后的有效引脚
%
% 输入：I_gray - 原始灰度图像
%       search_regions - 四侧搜索区域
%       body_boundary - 封装本体边界
% 输出：pins_regions - 各侧筛选后的引脚区域
%
% 使用方法：运行测试脚本 test_fig5_10_4

function pins_regions = QFP_Seg_Step4_GeometricFilter(I_gray, search_regions, body_boundary)
% 输入：I_gray - 原始灰度图像
%       search_regions - 四侧搜索区域
%       body_boundary - 封装本体边界
% 输出：pins_regions - 各侧筛选后的引脚区域

%% 参数设置
gamma_thr = 3;  % 长宽比阈值（引脚为细长矩形）
A_pin_min = 100;  % 引脚最小面积（像素）
A_pin_max = 5000;  % 引脚最大面积（像素）
delta_theta = 25;  % 方向角容差（度）
L_pin_min = 30;  % 引脚最小长度（像素）
L_pin_max = 150;  % 引脚最大长度（像素）

sides = {'top', 'bottom', 'left', 'right'};
pins_regions = struct();

% 用于可视化的图像
I_binary_all = cell(4, 1);
I_filtered_aspect = cell(4, 1);
I_filtered_angle = cell(4, 1);
I_filtered_position = cell(4, 1);

%% 处理四侧
for s = 1:length(sides)
    side_name = sides{s};
    fprintf('\n处理%s引脚...\n', side_name);
    
    region = search_regions.(side_name);
    mask_search = region.mask;
    
    %% 1. 局部Otsu阈值分割（提取高灰度区域 - 金属引脚）
    I_region = I_gray .* double(mask_search);
    I_region(I_region == 0) = NaN;
    
    valid_pixels = I_region(~isnan(I_region));
    if isempty(valid_pixels)
        pins_regions.(side_name) = [];
        continue;
    end
    
    threshold = graythresh(valid_pixels);
    I_binary = I_region > threshold;
    I_binary(isnan(I_region)) = false;
    I_binary_all{s} = I_binary;
    
    %% 2. 连通域分析
    CC = bwconncomp(I_binary);
    props = regionprops(CC, 'Area', 'Perimeter', 'Centroid', ...
                        'MajorAxisLength', 'MinorAxisLength', 'Orientation', 'PixelIdxList');
    
    if CC.NumObjects == 0
        pins_regions.(side_name) = [];
        continue;
    end
    
    fprintf('  检测到 %d 个候选区域\n', CC.NumObjects);
    
    %% 3. 几何约束筛选
    % 约束1：长宽比和面积筛选
    valid_aspect = false(CC.NumObjects, 1);
    for i = 1:CC.NumObjects
        area = props(i).Area;
        major_axis = props(i).MajorAxisLength;
        minor_axis = props(i).MinorAxisLength;
        
        if minor_axis > 0
            aspect_ratio = major_axis / minor_axis;
        else
            aspect_ratio = 0;
        end
        
        if aspect_ratio > gamma_thr && area >= A_pin_min && area <= A_pin_max
            valid_aspect(i) = true;
        end
    end
    fprintf('  长宽比与面积筛选: %d -> %d\n', CC.NumObjects, sum(valid_aspect));
    
    % 生成筛选后的图像
    I_filtered_aspect{s} = false(size(I_binary));
    for i = find(valid_aspect)'
        I_filtered_aspect{s}(props(i).PixelIdxList) = true;
    end
    
    % 约束2：方向角筛选
    valid_angle = valid_aspect;
    for i = find(valid_aspect)'
        orientation = props(i).Orientation;  % -90到90度
        
        % 对于上下侧，引脚应垂直（接近±90度）
        % 对于左右侧，引脚应水平（接近0度）
        if strcmp(side_name, 'top') || strcmp(side_name, 'bottom')
            % 垂直方向
            angle_diff = min(abs(orientation - 90), abs(orientation + 90));
        else
            % 水平方向
            angle_diff = min(abs(orientation), abs(orientation - 180));
        end
        
        if angle_diff > delta_theta
            valid_angle(i) = false;
        end
    end
    fprintf('  方向角约束筛选: %d -> %d\n', sum(valid_aspect), sum(valid_angle));
    
    % 生成筛选后的图像
    I_filtered_angle{s} = false(size(I_binary));
    for i = find(valid_angle)'
        I_filtered_angle{s}(props(i).PixelIdxList) = true;
    end
    
    % 约束3：位置筛选（到本体边界的距离）
    rect = body_boundary.rect;
    x_left = rect(1);
    y_top = rect(2);
    x_right = rect(1) + rect(3);
    y_bottom = rect(2) + rect(4);
    
    valid_position = valid_angle;
    for i = find(valid_angle)'
        centroid = props(i).Centroid;  % [x, y]
        
        % 计算到对应边界的垂直距离
        if strcmp(side_name, 'top')
            dist_perp = y_top - centroid(2);
        elseif strcmp(side_name, 'bottom')
            dist_perp = centroid(2) - y_bottom;
        elseif strcmp(side_name, 'left')
            dist_perp = x_left - centroid(1);
        else  % right
            dist_perp = centroid(1) - x_right;
        end
        
        if dist_perp < L_pin_min || dist_perp > L_pin_max
            valid_position(i) = false;
        end
    end
    fprintf('  位置约束筛选: %d -> %d\n', sum(valid_angle), sum(valid_position));
    
    % 生成最终筛选后的图像
    I_filtered_position{s} = false(size(I_binary));
    valid_pins = [];
    for i = find(valid_position)'
        I_filtered_position{s}(props(i).PixelIdxList) = true;
        valid_pins = [valid_pins; props(i)];
    end
    
    pins_regions.(side_name) = valid_pins;
end

%% 可视化
figure('Position', [100, 100, 1400, 800], 'Color', 'w');

% (a) 局部阈值分割结果
subplot(2, 2, 1);
imshow(I_gray); hold on;
colors = [1 0 0; 0 1 0; 1 0 1; 0 1 1];
for s = 1:4
    if ~isempty(I_binary_all{s})
        h = imshow(repmat(I_binary_all{s}, [1 1 3]) .* reshape(colors(s,:), [1 1 3]));
        set(h, 'AlphaData', I_binary_all{s} * 0.4);
    end
end
title('(a) 局部阈值分割结果', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);

% (b) 长宽比与面积筛选
subplot(2, 2, 2);
imshow(I_gray); hold on;
for s = 1:4
    if ~isempty(I_filtered_aspect{s})
        h = imshow(repmat(I_filtered_aspect{s}, [1 1 3]) .* reshape(colors(s,:), [1 1 3]));
        set(h, 'AlphaData', I_filtered_aspect{s} * 0.5);
    end
end
title(sprintf('(b) 长宽比与面积筛选\n(长宽比>%.1f, 面积∈[%d,%d])', ...
    gamma_thr, A_pin_min, A_pin_max), ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);

% (c) 方向角约束筛选
subplot(2, 2, 3);
imshow(I_gray); hold on;
for s = 1:4
    if ~isempty(I_filtered_angle{s})
        h = imshow(repmat(I_filtered_angle{s}, [1 1 3]) .* reshape(colors(s,:), [1 1 3]));
        set(h, 'AlphaData', I_filtered_angle{s} * 0.5);
    end
end
title(sprintf('(c) 方向角约束筛选\n(角度容差=±%.1f°)', delta_theta), ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);

% (d) 位置约束后的有效引脚
subplot(2, 2, 4);
imshow(I_gray); hold on;
for s = 1:4
    if ~isempty(I_filtered_position{s})
        h = imshow(repmat(I_filtered_position{s}, [1 1 3]) .* reshape(colors(s,:), [1 1 3]));
        set(h, 'AlphaData', I_filtered_position{s} * 0.6);
    end
    % 绘制边界框
    side_name = sides{s};
    if isfield(pins_regions, side_name) && ~isempty(pins_regions.(side_name))
        for i = 1:length(pins_regions.(side_name))
            pin = pins_regions.(side_name)(i);
            [r, c] = ind2sub(size(I_gray), pin.PixelIdxList);
            x_min = min(c); x_max = max(c);
            y_min = min(r); y_max = max(r);
            rectangle('Position', [x_min, y_min, x_max-x_min, y_max-y_min], ...
                'EdgeColor', colors(s,:), 'LineWidth', 1.5);
        end
    end
end
title(sprintf('(d) 位置约束后的有效引脚\n(距离∈[%d,%d]像素)', L_pin_min, L_pin_max), ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
xlabel('像素 X', 'FontSize', 10); ylabel('像素 Y', 'FontSize', 10);

sgtitle('图5-10-4 基于几何约束的四侧引脚区域筛选', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 输出统计信息
fprintf('\n四侧引脚筛选完成：\n');
for s = 1:length(sides)
    side_name = sides{s};
    if isfield(pins_regions, side_name) && ~isempty(pins_regions.(side_name))
        fprintf('  %s: %d 个引脚\n', side_name, length(pins_regions.(side_name)));
    else
        fprintf('  %s: 0 个引脚\n', side_name);
    end
end

end
