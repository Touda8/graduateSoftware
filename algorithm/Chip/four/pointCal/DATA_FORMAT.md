# 共面度测量数据接口说明

## 数据格式

### 1. 点云数据

点云数据以三个独立的MAT文件存储：

- **X.mat** - X坐标矩阵 (H×W double)
  - 每个像素位置对应的X坐标值（单位：mm）
  - 与图像尺寸一致
  
- **Y.mat** - Y坐标矩阵 (H×W double)
  - 每个像素位置对应的Y坐标值（单位：mm）
  - 与图像尺寸一致
  
- **Z.mat** - Z坐标矩阵 (H×W double)
  - 每个像素位置对应的Z坐标值（单位：mm）
  - 无效点用0或NaN表示
  - 与图像尺寸一致

**示例代码：**
```matlab
% 加载点云
load('X.mat', 'X');  % X: 2048×2448 double
load('Y.mat', 'Y');  % Y: 2048×2448 double
load('Z.mat', 'Z');  % Z: 2048×2448 double

% 查看有效点数
valid_mask = (Z ~= 0) & ~isnan(Z);
fprintf('有效点数: %d\n', sum(valid_mask(:)));

% 查看Z值范围
fprintf('Z范围: [%.2f, %.2f] mm\n', min(Z(valid_mask)), max(Z(valid_mask)));
```

---

### 2. 掩膜数据

掩膜数据从引脚定位步骤结果加载：

**文件：** `segment/results/qfp_pin_localization_results.mat`

**包含变量：**
- `mask_chip` (H×W logical) - 芯片本体掩膜
- `mask_pins` (H×W logical) - 所有引脚掩膜
- `pin_regions` (struct) - 引脚区域详细信息
  - `.top` / `.bottom` / `.left` / `.right` - 各侧引脚信息
    - `.pins` (struct array) - 引脚数组
      - `[i].mask` (H×W logical) - 第i个引脚的掩膜
      - `[i].centroid` [cx, cy] - 质心坐标
      - `[i].bbox` [x, y, w, h] - 边界框
      - `[i].area` - 像素面积

**示例代码：**
```matlab
% 加载掩膜数据
segment_path = 'segment/results/qfp_pin_localization_results.mat';
load(segment_path, 'mask_chip', 'pin_regions');

% 使用芯片掩膜作为本体掩膜
mask_body = mask_chip;

% 查看引脚信息
fprintf('上侧引脚数: %d\n', pin_regions.top.num_pins);
fprintf('下侧引脚数: %d\n', pin_regions.bottom.num_pins);
fprintf('左侧引脚数: %d\n', pin_regions.left.num_pins);
fprintf('右侧引脚数: %d\n', pin_regions.right.num_pins);

% 访问第一个顶部引脚
first_top_pin = pin_regions.top.pins(1);
fprintf('第一个引脚质心: (%.1f, %.1f)\n', ...
        first_top_pin.centroid(1), first_top_pin.centroid(2));
```

---

### 3. 完整工作流示例

```matlab
%% 1. 加载所有数据
% 点云
load('X.mat', 'X');
load('Y.mat', 'Y');
load('Z.mat', 'Z');

% 掩膜
segment_path = 'segment/results/qfp_pin_localization_results.mat';
load(segment_path, 'mask_chip', 'pin_regions');
mask_body = mask_chip;

%% 2. 步骤1：本体曲面拟合
body_surface = fig5_11_1(X, Y, Z, mask_body);

%% 3. 步骤2：高度分层
params_height = struct('k_z', 2.0, 'n_bins', 50);
pins_bottom_candidates = fig5_11_2(X, Y, Z, pin_regions, params_height);

%% 4. 步骤3：法向量精化
params_normal = struct('r_local', 0.2, 'theta_thr', 30, 'cluster_dist', 0.3);
pins_bottom_refined = fig5_11_3(pins_bottom_candidates, body_surface, params_normal);

%% 5. 步骤4：共面度计算
params_fitting = struct('max_iter', 5, 'kappa', 1.5);
coplanarity_result = fig5_11_4(pins_bottom_refined, params_fitting);

%% 6. 输出结果
fprintf('共面度: %.4f mm\n', coplanarity_result.coplanarity_value);
```

---

## 数据生成说明

### 如何从相位解包结果生成X,Y,Z

假设已有相位解包结果和标定参数：

```matlab
%% 从相位生成点云
% 假设变量：
% - phase_h, phase_v: 水平和垂直相位矩阵
% - calib_params: 标定参数结构体

[H, W] = size(phase_h);

% 生成像素坐标网格
[u_mesh, v_mesh] = meshgrid(1:W, 1:H);

% 通过标定映射计算三维坐标
% 这里需要根据实际标定方法填充
X = ... % 从 phase_h, u_mesh, v_mesh 计算
Y = ... % 从 phase_v, u_mesh, v_mesh 计算
Z = ... % 从 phase_h, phase_v, calib_params 计算

% 保存为MAT文件
save('X.mat', 'X');
save('Y.mat', 'Y');
save('Z.mat', 'Z');

fprintf('点云数据已保存\n');
fprintf('  尺寸: %d × %d\n', H, W);
fprintf('  有效点数: %d\n', sum((Z~=0) & ~isnan(Z), 'all'));
```

---

## 数据约定

### 坐标系
- **X轴**: 通常对应图像列方向（水平向右）
- **Y轴**: 通常对应图像行方向（垂直向下）
- **Z轴**: 垂直于芯片表面向上（深度方向）
- **单位**: 毫米 (mm)

### 无效点表示
- **方法1**: Z = 0
- **方法2**: Z = NaN
- **推荐**: 同时检查 `(Z ~= 0) & ~isnan(Z)`

### 数据质量要求
- **分辨率**: ≥1024×1024像素
- **Z向精度**: ≤0.01mm（对于共面度测量）
- **有效点比例**: 芯片区域≥80%，引脚区域≥60%

---

## 故障排查

### 问题1: "矩阵尺寸不一致"
```matlab
% 检查尺寸
fprintf('X尺寸: %d × %d\n', size(X,1), size(X,2));
fprintf('Y尺寸: %d × %d\n', size(Y,1), size(Y,2));
fprintf('Z尺寸: %d × %d\n', size(Z,1), size(Z,2));
fprintf('掩膜尺寸: %d × %d\n', size(mask_body,1), size(mask_body,2));
```

### 问题2: "有效点数过少"
```matlab
% 统计有效点
valid = (Z ~= 0) & ~isnan(Z);
fprintf('总像素数: %d\n', numel(Z));
fprintf('有效点数: %d (%.1f%%)\n', sum(valid(:)), 100*sum(valid(:))/numel(Z));

% 检查本体区域覆盖
body_valid = valid & mask_body;
fprintf('本体有效点: %d / %d (%.1f%%)\n', ...
        sum(body_valid(:)), sum(mask_body(:)), ...
        100*sum(body_valid(:))/sum(mask_body(:)));
```

### 问题3: "Z值范围异常"
```matlab
% 检查Z值分布
valid = (Z ~= 0) & ~isnan(Z);
Z_valid = Z(valid);
fprintf('Z统计:\n');
fprintf('  最小值: %.4f mm\n', min(Z_valid));
fprintf('  最大值: %.4f mm\n', max(Z_valid));
fprintf('  均值: %.4f mm\n', mean(Z_valid));
fprintf('  标准差: %.4f mm\n', std(Z_valid));

% 可视化高度分布
figure;
histogram(Z_valid, 100);
xlabel('Z (mm)'); ylabel('频数');
title('高度分布直方图');
grid on;
```

---

## 兼容性说明

本数据格式与以下工具兼容：
- **引脚定位模块** (segment/) - 提供mask_chip和pin_regions
- **点云重建模块** - 生成X.mat/Y.mat/Z.mat
- **可视化工具** - 使用相同的坐标系统

**版本**: v1.0 (2026-02-14)
