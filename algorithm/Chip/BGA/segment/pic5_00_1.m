% pic5_00_1.m - 图5-0-1: 芯片区域粗定位流程与结果
% (a) 原始灰度图像
% (b) 大津阈值二值分割结果
% (c) 形态学处理后图像
% (d) 最大连通域提取结果
% (e) 掩膜相乘后的芯片区域
% 输入: bga.bmp

clear; clc; close all;

%% 读取原始图像
I_original = imread('bga.bmp');
if size(I_original, 3) == 3
    I_original = rgb2gray(I_original);
end
I_original = double(I_original);

%% 创建图形
fig = figure('Position', [50, 50, 1600, 600], 'Color', 'w');

%% (a) 原始灰度图像
subplot(2, 3, 1);
imshow(uint8(I_original), []);
% title('(a) 原始灰度图像', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
colormap(gca, gray);

%% (b) 大津阈值二值分割结果
subplot(2, 3, 2);
% 1. Otsu全局二值分割
level_chip = graythresh(uint8(I_original));
level_chip = 0.74;
BW_chip = imbinarize(uint8(I_original), level_chip);

imshow(BW_chip);
% title('(b) 大津阈值二值分割结果', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
% text(10, 30, sprintf('阈值: %.3f', level_chip), ...
%     'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold', ...
%     'BackgroundColor', [0, 0, 0, 0.6]);

%% (c) 形态学处理后图像
subplot(2, 3, 3);
% 2. 形态学操作：先腐蚀后膨胀（开运算，使用50×50正方形结构元素）
se_chip = strel('square', 50);
BW_eroded = imerode(BW_chip, se_chip);
BW_morph = imdilate(BW_eroded, se_chip);

imshow(BW_morph, []);
% title('(c) 形态学处理后图像', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
% text(10, 30, '结构元素: 50×50方形', ...
%     'Color', 'y', 'FontSize', 10, 'FontWeight', 'bold', ...
%     'BackgroundColor', [0, 0, 0, 0.6]);
% text(10, 60, '操作: 先腐蚀后膨胀', ...
%     'Color', 'y', 'FontSize', 10, 'FontWeight', 'bold', ...
%     'BackgroundColor', [0, 0, 0, 0.6]);

%% (d) 最大连通域提取结果
subplot(2, 3, 4);
% 3. 反转后连通域分析，保留最大连通域
BW_inverted = ~BW_morph;
CC_chip = bwconncomp(BW_inverted);
stats_chip = regionprops(CC_chip, 'Area');

% 找到最大连通域
[max_area, maxIdx_chip] = max([stats_chip.Area]);
BW_max_chip = false(size(BW_inverted));
BW_max_chip(CC_chip.PixelIdxList{maxIdx_chip}) = true;

imshow(BW_max_chip, []);
hold on;
% 绘制边界轮廓
boundaries = bwboundaries(BW_max_chip);
for k = 1:length(boundaries)
    boundary = boundaries{k};
    plot(boundary(:,2), boundary(:,1), 'g-', 'LineWidth', 2);
end
% title('(d) 最大连通域提取结果', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
% text(10, 30, sprintf('连通域数: %d', CC_chip.NumObjects), ...
%     'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold', ...
%     'BackgroundColor', [0, 0, 0, 0.6]);
% text(10, 60, sprintf('最大面积: %d像素', round(max_area)), ...
%     'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold', ...
%     'BackgroundColor', [0, 0, 0, 0.6]);

%% (e) 掩膜相乘后的芯片区域
subplot(2, 3, 5);
% 4. 制作掩膜：连通域区域为1，其他为-1
mask_chip = ones(size(I_original)) * (-1);
mask_chip(BW_max_chip) = 1;

% 5. 掩膜与原始灰度图相乘
I_masked = I_original .* mask_chip;

% 显示结果（将负值区域设为0用于显示）
I_display = I_masked;
I_display(I_display < 0) = 0;
imshow(uint8(I_display), []);
% title('(e) 掩膜相乘后的芯片区域', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');

% 计算有效区域占比
valid_pixels = sum(I_masked(:) >= 0);
total_pixels = numel(I_masked);
% text(10, 30, sprintf('有效区域: %.1f%%', valid_pixels/total_pixels*100), ...
%     'Color', 'c', 'FontSize', 10, 'FontWeight', 'bold', ...
%     'BackgroundColor', [0, 0, 0, 0.6]);

%% 添加处理流程示意图
% subplot(2, 3, 6);
% axis off;
% hold on;
% xlim([0, 10]);
% ylim([0, 10]);
% 
% % 绘制流程框图
% steps = {'原始图像', 'Otsu分割', '形态学处理', '连通域提取', '掩膜相乘'};
% y_positions = [9, 7.5, 6, 4.5, 3];
% 
% for i = 1:length(steps)
%     % 绘制流程框
%     rectangle('Position', [2, y_positions(i)-0.4, 6, 0.8], ...
%               'FaceColor', [0.9, 0.95, 1], ...
%               'EdgeColor', [0.3, 0.5, 0.9], ...
%               'LineWidth', 1.5);
% 
%     % 添加文字
%     text(5, y_positions(i), steps{i}, ...
%          'HorizontalAlignment', 'center', ...
%          'VerticalAlignment', 'middle', ...
%          'FontSize', 10, ...
%          'FontWeight', 'bold', ...
%          'FontName', 'SimHei');
% 
%     % 绘制箭头（除最后一个）
%     if i < length(steps)
%         annotation('arrow', [0.84, 0.84], ...
%                    [(y_positions(i)-0.4)/10, (y_positions(i+1)+0.4)/10], ...
%                    'LineWidth', 2, 'HeadLength', 6, 'HeadWidth', 6, ...
%                    'Color', [0.3, 0.5, 0.9]);
%     end
% end
% 
% % 添加子图标题
% text(5, 9.8, '粗定位流程', ...
%      'HorizontalAlignment', 'center', ...
%      'FontSize', 11, ...
%      'FontWeight', 'bold', ...
%      'FontName', 'SimHei', ...
%      'Color', [0.2, 0.4, 0.8]);
% 
% % 添加关键参数说明
% text(5, 1.5, '关键参数:', ...
%      'HorizontalAlignment', 'center', ...
%      'FontSize', 9, ...
%      'FontWeight', 'bold', ...
%      'Color', [0.6, 0, 0]);
% text(5, 0.8, '结构元素: 50×50方形', ...
%      'HorizontalAlignment', 'center', ...
%      'FontSize', 8);
% text(5, 0.3, '掩膜值: 芯片=1, 背景=-1', ...
%      'HorizontalAlignment', 'center', ...
%      'FontSize', 8);

%% 添加总标题
sgtitle('图5-0-1 芯片区域粗定位流程与结果', ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimHei');

%% 保存图片
saveas(fig, 'fig5_00_1_芯片区域粗定位.png');
saveas(fig, 'fig5_00_1_芯片区域粗定位.fig');
fprintf('图5-0-1生成完成\n');
fprintf('芯片区域定位成功，有效区域占比: %.2f%%\n', valid_pixels/total_pixels*100);
