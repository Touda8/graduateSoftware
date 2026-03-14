# allDeal_function 使用说明

## 功能概述

`allDeal_function.m` 是 `allDeal.m` 的函数化版本，用于焊球检测、3D测量和高度分析。

## 函数签名

```matlab
[ballResults, figHandles] = allDeal_function(bga_image_path, X, Y, Z, output_dir, config)
```

## 输入参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `bga_image_path` | string | BGA图像文件路径（例如：第50张图） |
| `X` | matrix | X坐标矩阵（与图像尺寸相同） |
| `Y` | matrix | Y坐标矩阵（与图像尺寸相同） |
| `Z` | matrix | Z坐标矩阵（高度数据，例如 mix_Z） |
| `output_dir` | string | 输出图片保存目录（可选） |
| `config` | struct | 配置参数（可选） |

## 输出结果

| 参数 | 类型 | 说明 |
|------|------|------|
| `ballResults` | struct array | 每个焊球的测量结果 |
| `figHandles` | array | 生成的图形句柄数组 |

### ballResults 结构体字段

- `order` - 排序编号（1, 2, 3, ...）
- `xc, yc` - 圆心坐标（像素）
- `r` - 半径（像素）
- `x3d, y3d, z3d` - 3D顶点坐标（mm）
- `height` - 相对基板高度（mm）
- `row` - 所在行号
- `success` - 是否测量成功（true/false）

## 配置参数 (config)

```matlab
config = struct();
config.circularity_threshold = 0.85;      % 圆形度阈值
config.area_min = 300;                     % 最小面积（像素²）
config.area_max = 4000;                    % 最大面积（像素²）
config.edge_distance_threshold = 1;        % 边界距离阈值
config.circle_fit_iterations = 10;         % 圆拟合迭代次数
config.circle_fit_beta = 2.0;              % 圆拟合阈值系数
config.substrate_downsample_ratio = 0.5;   % 基板降采样比例
config.substrate_angle_threshold = 30;     % 基板法向量角度阈值（度）
config.ball_angle_threshold = 45;          % 焊球法向量角度阈值（度）
config.surface_fit_iterations = 100;       % 曲面拟合迭代次数
config.surface_fit_sigma = 2.5;            % 曲面拟合sigma系数
config.z_flip = 160;                       % Z轴翻转值（0表示不翻转）
```

## 使用示例

### 示例1：基本使用

```matlab
% 准备数据
bga_image = 'path/to/bga.bmp';
load('X.mat');
load('Y.mat');
load('Z.mat');
output_dir = 'output/results';

% 调用函数
[ballResults, figs] = allDeal_function(bga_image, X, Y, Z, output_dir);

% 查看结果
for i = 1:length(ballResults)
    if ballResults(i).success
        fprintf('焊球 #%d: 高度 = %.4f mm\n', ...
                ballResults(i).order, ballResults(i).height);
    end
end
```

### 示例2：在 reconstruction_SNR_BGA.m 中使用

```matlab
% 第50张图作为 BGA 图像
bga_image_path = strcat('..\..\experiment\',data,'\thing\',ResName,'\', ...
                        num2str(ProNum),'\',num2str(index),'\', num2str(50),'.bmp');

% 输出目录
ball_output_dir = strcat('..\..\experiment\',data,'\thing\',ResName,'\info\ball_results_', num2str(index));

% 配置参数
config = struct();
config.z_flip = 0;  % mix_Z 已处理过，不需要翻转

% 调用函数
[ballResults, ~] = allDeal_function(bga_image_path, X, Y, mix_Z, ball_output_dir, config);
```

### 示例3：自定义配置

```matlab
% 自定义配置
config = struct();
config.circularity_threshold = 0.90;  % 更严格的圆形度要求
config.area_min = 500;                % 更大的最小面积
config.surface_fit_iterations = 150;  % 更多迭代次数

[ballResults, ~] = allDeal_function(bga_image, X, Y, Z, output_dir, config);
```

## 生成的输出文件

在指定的 `output_dir` 目录下会生成：

1. **焊球分割与排序.png**
   - 左图：焊球分割结果（圆拟合）
   - 右图：焊球排序编号（按行列排序）

2. **焊球3D分布.png**
   - 3D散点图显示所有焊球顶点
   - 颜色映射高度值
   - 包含基板曲面

## 与 reconstruction_SNR_BGA.m 的集成

修改后的 `reconstruction_SNR_BGA.m` 会：

1. 对每个 `index`（组）调用 `allDeal_function`
2. 使用第50张图作为 BGA 图像
3. 使用 `mix_Z` 作为高度数据
4. 将所有 `index` 的结果汇总到 Excel：
   - 每列对应一个 `index`
   - 每行对应一个焊球编号
   - 最后一行显示每列的范围（Max - Min）

### Excel 输出格式

| 焊球编号 | Index_1_高度(mm) | Index_2_高度(mm) | ... |
|----------|------------------|------------------|-----|
| Ball_1   | 0.5234           | 0.5241           | ... |
| Ball_2   | 0.5198           | 0.5203           | ... |
| ...      | ...              | ...              | ... |
| 范围(Max-Min) | 0.0123      | 0.0118           | ... |

## 注意事项

1. **Z轴方向**：如果 Z 已经在外部处理过（如 `mix_Z`），设置 `config.z_flip = 0`
2. **图像路径**：确保 BGA 图像路径正确且文件存在
3. **数据尺寸**：X, Y, Z 矩阵必须与图像尺寸一致
4. **内存占用**：处理多个 index 时注意内存使用
5. **错误处理**：函数内部有 try-catch，失败时会返回空结果而不中断

## 常见问题

**Q: 为什么某些焊球测量失败？**
A: 可能原因：
- 点云数据不足（< 50个点）
- 法向量角度超出阈值
- Hessian 矩阵检查失败
- 极值点偏离焊球区域

**Q: 如何调整测量精度？**
A: 调整以下参数：
- `surface_fit_iterations`：增加迭代次数
- `surface_fit_sigma`：减小 sigma 系数（更严格）
- `ball_angle_threshold`：调整法向量阈值

**Q: Excel 文件写入失败？**
A: 检查：
- 文件是否被其他程序打开
- 是否有写入权限
- 路径是否存在

## 更新日志

- **2026-02-04**：初始版本，从 allDeal.m 创建函数化版本
