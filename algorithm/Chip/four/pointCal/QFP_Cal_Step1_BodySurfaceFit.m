% fig5_11_1.m - 图5-11-1: 封装本体顶面曲面拟合与法向量提取
%
% 功能：实现扁平封装芯片本体顶面的二阶多项式曲面拟合并提取主法向量
%
% 解决问题：
%   1. 封装本体可能存在轻微翘曲，需用二阶曲面描述
%   2. 得到封装本体法向量作为后续引脚法向量筛选的参考方向
%   3. 评估封装本体平整度，判断是否存在变形
%
% 核心方法：
%   二阶多项式曲面拟合：
%     Z_body(X,Y) = s0 + s1*X + s2*Y + s3*X^2 + s4*Y^2 + s5*XY
%     通过最小二乘法求解6个系数
%   主法向量提取：
%     从一阶项系数计算归一化法向量 n_body = (s1, s2, -1) / ||...||
%   拟合质量评估：
%     计算拟合残差RMSE，分析点云分布特征
%
% 输入：
%   X - X坐标矩阵 (H×W)，每个像素的X坐标
%   Y - Y坐标矩阵 (H×W)，每个像素的Y坐标
%   Z - Z坐标矩阵 (H×W)，每个像素的Z坐标，无效点为0或NaN
%   mask_body - 封装本体掩膜 (H×W logical)
%
% 输出：
%   body_surface - 封装本体曲面信息结构体：
%     .coeffs - 曲面参数 [s0 s1 s2 s3 s4 s5]
%     .normal_vector - 主法向量 [nx ny nz]
%     .centroid - 点云质心 [x_c y_c z_c]
%     .rmse - 拟合残差均方根
%     .point_cloud - 本体点云 (N×3，列为X,Y,Z)
%     .residuals - 拟合残差向量 (N×1)
%
% 使用方法：
%   load('X.mat', 'X'); load('Y.mat', 'Y'); load('Z.mat', 'Z');
%   load('mask_chip.mat', 'mask_chip');  % 从segment步骤获取
%   body_surface = fig5_11_1(X, Y, Z, mask_chip);
%   或运行测试脚本：test_fig5_11_1

function body_surface = QFP_Cal_Step1_BodySurfaceFit(X, Y, Z, mask_body)

fprintf('====== 图5-11-1: 封装本体曲面拟合与法向量提取 ======\n');

%% 1. 验证输入数据
if ~isnumeric(X) || ~isnumeric(Y) || ~isnumeric(Z)
    error('X, Y, Z必须为数值矩阵');
end
if ~isequal(size(X), size(Y), size(Z))
    error('X, Y, Z矩阵尺寸必须一致');
end
if ~islogical(mask_body)
    mask_body = logical(mask_body);
end

[H, W] = size(Z);
fprintf('点云尺寸: %d × %d\n', H, W);
fprintf('本体掩膜像素数: %d\n', sum(mask_body(:)));

% 对掩膜进行矩形腐蚀，去除边缘杂点
se = strel('rectangle', [50, 50]);
mask_body_eroded = imerode(mask_body, se);
n_removed = sum(mask_body(:)) - sum(mask_body_eroded(:));
fprintf('矩形腐蚀去除边缘像素: %d (腐蚀核: 1×1)\n', n_removed);
mask_body = mask_body_eroded;

%% 2. 提取封装本体点云
X_all = X(mask_body);
Y_all = Y(mask_body);
Z_all = Z(mask_body);

% 剔除无效点（Z=0或NaN）
valid_idx = (Z_all ~= 0) & ~isnan(Z_all) & ~isnan(X_all) & ~isnan(Y_all);
X_body = X_all(valid_idx);
Y_body = Y_all(valid_idx);
Z_body = Z_all(valid_idx);
N_body = length(Z_body);

if N_body < 100
    error('本体有效点云数量不足（< 100），无法拟合！');
end

fprintf('本体有效点云数: %d\n', N_body);
fprintf('X范围: [%.2f, %.2f] mm\n', min(X_body), max(X_body));
fprintf('Y范围: [%.2f, %.2f] mm\n', min(Y_body), max(Y_body));
fprintf('Z范围: [%.2f, %.2f] mm\n', min(Z_body), max(Z_body));

%% 3. 构建二阶多项式曲面拟合矩阵
% Z = s0 + s1*X + s2*Y + s3*X^2 + s4*Y^2 + s5*XY
fprintf('\n构建二阶曲面拟合矩阵...\n');
M_body = [ones(N_body, 1), X_body(:), Y_body(:), ...
          X_body(:).^2, Y_body(:).^2, X_body(:).*Y_body(:)];

%% 4. 最小二乘法求解曲面参数
fprintf('最小二乘法求解曲面参数...\n');
coeffs = (M_body' * M_body) \ (M_body' * Z_body(:));

fprintf('曲面参数:\n');
fprintf('  s0 (常数项) = %.6f\n', coeffs(1));
fprintf('  s1 (X一阶) = %.6f\n', coeffs(2));
fprintf('  s2 (Y一阶) = %.6f\n', coeffs(3));
fprintf('  s3 (X二阶) = %.6e\n', coeffs(4));
fprintf('  s4 (Y二阶) = %.6e\n', coeffs(5));
fprintf('  s5 (XY交叉项) = %.6e\n', coeffs(6));

%% 5. 计算主法向量
% 从一阶项提取法向量 n = (s1, s2, -1)
nx_raw = coeffs(2);
ny_raw = coeffs(3);
nz_raw = -1;
norm_magnitude = sqrt(nx_raw^2 + ny_raw^2 + nz_raw^2);
normal_vector = [nx_raw, ny_raw, nz_raw] / norm_magnitude;

fprintf('\n主法向量:\n');
fprintf('  n = [%.6f, %.6f, %.6f]\n', normal_vector);
fprintf('  与Z轴夹角: %.2f°\n', acosd(abs(normal_vector(3))));

%% 6. 计算拟合残差
Z_fitted = M_body * coeffs;
residuals = Z_body(:) - Z_fitted;
rmse = sqrt(mean(residuals.^2));
max_residual = max(abs(residuals));

fprintf('\n拟合质量评估:\n');
fprintf('  RMSE = %.4f mm\n', rmse);
fprintf('  最大残差 = %.4f mm\n', max_residual);
fprintf('  残差标准差 = %.4f mm\n', std(residuals));

%% 7. 计算点云质心
centroid = [mean(X_body), mean(Y_body), mean(Z_body)];
fprintf('\n点云质心: [%.2f, %.2f, %.2f]\n', centroid);

%% 8. 组装输出结构体
body_surface = struct();
body_surface.coeffs = coeffs;
body_surface.normal_vector = normal_vector;
body_surface.centroid = centroid;
body_surface.rmse = rmse;
body_surface.point_cloud = [X_body(:), Y_body(:), Z_body(:)];
body_surface.residuals = residuals;
body_surface.Z_fitted = Z_fitted;

%% 9. 可视化
fprintf('\n生成可视化结果...\n');
visualize_body_surface(body_surface);

fprintf('====== 封装本体曲面拟合完成 ======\n\n');

end

%% ========== 辅助函数：可视化 ==========
function visualize_body_surface(body_surface)

figure('Position', [100, 100, 1600, 900], 'Color', 'w', 'Name', '图5-11-1');

pc = body_surface.point_cloud;
X_body = pc(:,1);
Y_body = pc(:,2);
Z_body = pc(:,3);
Z_fitted = body_surface.Z_fitted;
residuals = body_surface.residuals;
coeffs = body_surface.coeffs;
normal_vec = body_surface.normal_vector;
centroid = body_surface.centroid;

%% 子图(a): 封装本体点云分布
subplot(2, 2, 1);
scatter3(X_body, Y_body, Z_body, 15, Z_body, 'filled');
colormap(gca, jet);
colorbar('FontSize', 10);
xlabel('X (mm)', 'FontSize', 11, 'FontName', 'SimHei');
ylabel('Y (mm)', 'FontSize', 11, 'FontName', 'SimHei');
zlabel('Z (mm)', 'FontSize', 11, 'FontName', 'SimHei');
% title('(a) 封装本体点云分布', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on; view(45, 30);

%% 子图(b): 二阶曲面拟合结果
subplot(2, 2, 2);
% 绘制原始点云
scatter3(X_body, Y_body, Z_body, 10, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.3);
hold on;

% 生成曲面网格
x_range = linspace(min(X_body), max(X_body), 50);
y_range = linspace(min(Y_body), max(Y_body), 50);
[X_grid, Y_grid] = meshgrid(x_range, y_range);
Z_grid = coeffs(1) + coeffs(2)*X_grid + coeffs(3)*Y_grid + ...
         coeffs(4)*X_grid.^2 + coeffs(5)*Y_grid.^2 + coeffs(6)*X_grid.*Y_grid;

% 绘制拟合曲面
surf(X_grid, Y_grid, Z_grid, 'FaceAlpha', 0.7, 'EdgeColor', 'none', 'FaceColor', 'interp');
colormap(gca, jet);
colorbar('FontSize', 10);

% 绘制法向量箭头
arrow_length = (max(X_body) - min(X_body)) * 0.3;
quiver3(centroid(1), centroid(2), centroid(3), ...
        normal_vec(1)*arrow_length, normal_vec(2)*arrow_length, normal_vec(3)*arrow_length, ...
        'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);

xlabel('X (mm)', 'FontSize', 11, 'FontName', 'SimHei');
ylabel('Y (mm)', 'FontSize', 11, 'FontName', 'SimHei');
zlabel('Z (mm)', 'FontSize', 11, 'FontName', 'SimHei');
% title(sprintf('(b) 二阶曲面拟合结果\nRMSE=%.4fmm', body_surface.rmse), ...
%       'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on; view(45, 30);
legend({'原始点云', '拟合曲面', '法向量'}, 'Location', 'best', 'FontSize', 9, 'FontName', 'SimHei');

%% 子图(c): 拟合残差伪彩色图
subplot(2, 2, 3);
scatter3(X_body, Y_body, Z_body, 25, residuals, 'filled');
colormap(gca, jet);
cb = colorbar('FontSize', 10);
ylabel(cb, '残差 (mm)', 'FontSize', 10, 'FontName', 'SimHei');
caxis([-max(abs(residuals)), max(abs(residuals))]);  % 对称色标
xlabel('X (mm)', 'FontSize', 11, 'FontName', 'SimHei');
ylabel('Y (mm)', 'FontSize', 11, 'FontName', 'SimHei');
zlabel('Z (mm)', 'FontSize', 11, 'FontName', 'SimHei');
% title(sprintf('(c) 拟合残差伪彩色图\n最大残差=%.4fmm', max(abs(residuals))), ...
%       'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on; view(45, 30);

%% 子图(d): 法向量方向标识与残差分布
subplot(2, 2, 4);
% 左子图：法向量方向
% ax1 = axes('Position', [0.55, 0.1, 0.18, 0.35]);
% 绘制单位球面参考
[sphere_x, sphere_y, sphere_z] = sphere(20);
surf(sphere_x*0.5, sphere_y*0.5, sphere_z*0.5, 'FaceAlpha', 0.1, ...
     'EdgeColor', [0.7 0.7 0.7], 'FaceColor', [0.9 0.9 0.9]);
hold on;
% 绘制法向量
quiver3(0, 0, 0, normal_vec(1), normal_vec(2), normal_vec(3), ...
        'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 绘制坐标轴
quiver3(0, 0, 0, 1, 0, 0, 'k--', 'LineWidth', 1.5, 'MaxHeadSize', 0.3);
quiver3(0, 0, 0, 0, 1, 0, 'k--', 'LineWidth', 1.5, 'MaxHeadSize', 0.3);
quiver3(0, 0, 0, 0, 0, 1, 'k--', 'LineWidth', 1.5, 'MaxHeadSize', 0.3);
text(1.1, 0, 0, 'X', 'FontSize', 11, 'FontWeight', 'bold');
text(0, 1.1, 0, 'Y', 'FontSize', 11, 'FontWeight', 'bold');
text(0, 0, 1.1, 'Z', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
% title('法向量方向', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
axis equal; grid on; view(120, 25);
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);

% % 右子图：残差分布直方图
% ax2 = axes('Position', [0.75, 0.1, 0.18, 0.35]);
% histogram(residuals, 30, 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'k', 'LineWidth', 0.8);
% xlabel('残差 (mm)', 'FontSize', 10, 'FontName', 'SimHei');
% ylabel('点数', 'FontSize', 10, 'FontName', 'SimHei');
% title(sprintf('残差分布直方图\n均值=%.4f, 标准差=%.4f', mean(residuals), std(residuals)), ...
%       'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimHei');
grid on;

% 总标题
sgtitle('图5-11-1: 封装本体顶面曲面拟合与法向量提取', ...
        'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimHei');

end
