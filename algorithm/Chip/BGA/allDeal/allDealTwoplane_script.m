% allDealTwoplane_script.m - 修改版，直接运行脚本调试
% 该脚本基于 allDealTwoplane.m，调整为直接运行的形式

clear; clc; close all;

%% 输入配置
% 输入文件路径
INPUT_IMAGE = '1.bmp';        % 2D图像
INPUT_X = '../pointCal/X.mat';             % X坐标
INPUT_Y = '../pointCal/Y.mat';             % Y坐标  
INPUT_Z = 'Z.mat';             % Z坐标

% 输出文件路径
OUTPUT_DIR = './output';

% 算法参数配置
CONFIG = struct();
CONFIG.circularity_threshold = 0.85;      % 圆形度阈值
CONFIG.area_min = 300;                     % 最小面积
CONFIG.area_max = 4000;                    % 最大面积
CONFIG.edge_distance_threshold = 1;        % 边界距离阈值
CONFIG.circle_fit_iterations = 10;         % 圆拟合迭代次数
CONFIG.circle_fit_beta = 2.0;              % 圆拟合阈值系数
CONFIG.substrate_downsample_ratio = 0.5;   % 基板降采样比例
CONFIG.substrate_angle_threshold = 30;     % 基板法向量角度阈值(度)
CONFIG.ball_angle_threshold = 45;          % 焊球法向量角度阈值(度)
CONFIG.surface_fit_iterations = 100;       % 曲面拟合迭代次数
CONFIG.surface_fit_sigma = 2.5;            % 曲面拟合sigma系数
CONFIG.z_flip = 160;                       % Z轴翻转值

%% 加载数据
fprintf('加载输入数据...\n');
I_original = imread(INPUT_IMAGE);
if size(I_original, 3) == 3
    I_original = rgb2gray(I_original);
end

load(INPUT_X, 'X');
load(INPUT_Y, 'Y');
load(INPUT_Z, 'Z');

%% 调用 allDealTwoplane 函数
fprintf('调用 allDealTwoplane 函数...\n');
[ballResults, figHandles] = allDealTwoplane(I_original, X, Y, Z, OUTPUT_DIR, CONFIG);

fprintf('处理完成！\n');