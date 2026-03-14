# QFP芯片引脚共面度测量算法

基于论文第5.3.2节"扁平封装芯片引脚共面度测量"理论实现的模块化算法框架。

## 算法概述

本模块实现了基于结构光三维测量的QFP芯片引脚共面度测量算法，包含4个核心步骤：

1. **封装本体顶面曲面拟合与法向量提取** (图5-11-1)
2. **基于高度分层的引脚底部接触区提取** (图5-11-2)
3. **基于法向量约束与空间连通性的点云精化** (图5-11-3)
4. **引脚底部接触面平面拟合与共面度计算** (图5-11-4/5)

## 文件结构

```
pointCal/
├── qfp_coplanarity_main.m          # 主控制脚本（orchestrates完整流程）
├── test_qfp_coplanarity_all.m     # 完整端到端测试脚本
│
├── fig5_11_1.m                     # 步骤1：本体曲面拟合
├── fig5_11_2.m                     # 步骤2：高度分层提取
├── fig5_11_3.m                     # 步骤3：法向量约束精化
├── fig5_11_4.m                     # 步骤4：平面拟合与共面度
│
├── test_fig5_11_1.m                # 步骤1单元测试
├── test_fig5_11_2.m                # 步骤2单元测试
├── test_fig5_11_3.m                # 步骤3单元测试
├── test_fig5_11_4.m                # 步骤4单元测试
│
└── results/                         # 结果输出目录
    ├── qfp_coplanarity_results.mat        # MATLAB数据文件
    ├── coplanarity_report.txt             # 测量报告
    └── *.png                               # 可视化图像
```

## 快速开始

### 1. 运行完整测试（使用模拟数据）

```matlab
% 端到端测试所有4个步骤
test_qfp_coplanarity_all
```

### 2. 单元测试各个模块

```matlab
% 测试步骤1：本体曲面拟合
test_fig5_11_1

% 测试步骤2：高度分层
test_fig5_11_2

% 测试步骤3：法向量精化
test_fig5_11_3

% 测试步骤4：共面度计算
test_fig5_11_4
```

### 3. 使用真实数据

```matlab
% 加载点云数据
load('X.mat', 'X');  % X坐标矩阵 (H×W)
load('Y.mat', 'Y');  % Y坐标矩阵 (H×W)
load('Z.mat', 'Z');  % Z坐标矩阵 (H×W)

% 加载引脚定位结果
load('qfp_pin_localization_results.mat', 'mask_chip', 'pin_regions');

% 运行共面度测量
qfp_coplanarity_main
```

## 算法详细说明

### 步骤1: 封装本体曲面拟合

**输入:**
- `X` - X坐标矩阵 (H×W），每个像素的X坐标
- `Y` - Y坐标矩阵 (H×W），每个像素的Y坐标  
- `Z` - Z坐标矩阵 (H×W），每个像素的Z坐标
- `mask_body` - 封装本体区域掩膜 (H×W logical)

**输出:**
- `body_surface` - 结构体包含：
  - `coeffs[6]` - 曲面拟合系数 [s0, s1, s2, s3, s4, s5]
  - `normal_vector[3]` - 单位法向量
  - `rmse` - 拟合均方根误差
  - `point_cloud[N×3]` - 本体点云
  - `residuals[N]` - 拟合残差

**数学模型:**
$$Z_{body}(X,Y) = s_0 + s_1X + s_2Y + s_3X^2 + s_4Y^2 + s_5XY$$

**使用示例:**
```matlab
load('X.mat', 'X'); load('Y.mat', 'Y'); load('Z.mat', 'Z');
load('qfp_pin_localization_results.mat', 'mask_chip');
body_surface = fig5_11_1(X, Y, Z, mask_chip);
fprintf('法向量: [%.4f, %.4f, %.4f]\n', body_surface.normal_vector);
```

---

### 步骤2: 高度分层提取

**输入:**
- `X`, `Y`, `Z` - 点云坐标矩阵 (H×W)
- `pin_regions` - 引脚区域信息（从fig5_10_3获取）
- `params` - 参数结构体：
  - `k_z` (default=2.0) - 阈值系数
  - `n_bins` (default=50) - 直方图分组数

**输出:**
- `pins_bottom_candidates[N]` - 数组，每个元素包含：
  - `pin_id` - 引脚ID
  - `side` - 所属侧 ('top'/'bottom'/'left'/'right')
  - `point_cloud_full[M×3]` - 完整引脚点云
  - `point_cloud_bottom[K×3]` - 底部候选点云
  - `z_threshold` - 自适应阈值

**核心算法:**
1. 对引脚点云高度Z进行直方图统计
2. 识别低高度区域（最低30%分组）
3. 计算自适应阈值：$T_z = \text{median}(Z_{low}) + k_z \cdot \sigma(Z_{low})$
4. 提取 $Z < T_z$ 的点作为底部候选

**使用示例:**
```matlab
load('qfp_pin_localization_results.mat', 'pin_regions');
params = struct('k_z', 2.0, 'n_bins', 50);
pins_bottom = fig5_11_2(X, Y, Z, pin_regions, params);
```

---

### 步骤3: 法向量约束精化

**输入:**
- `pins_bottom_candidates` - 步骤2输出的底部候选点云
- `body_surface` - 步骤1输出的本体曲面信息
- `params` - 参数结构体：
  - `r_local` (default=0.2mm) - 局部邻域半径
  - `theta_thr` (default=30°) - 法向量夹角阈值
  - `cluster_dist` (default=0.3mm) - 连通性聚类距离

**输出:**
- `pins_bottom_refined[N]` - 精化后的引脚数组，包含：
  - `point_cloud_bottom_refined[K×3]` - 精化点云
  - `outliers[M×3]` - 被剔除的异常点
  - `local_normals[N×3]` - 局部法向量
  - `normal_angles[N]` - 与本体法向量夹角
  - `n_clusters` - 连通域数量

**核心算法:**
1. 对每个点计算局部协方差矩阵：$C_j = \frac{1}{|N_j|}\sum_{p_k \in N_j}(p_k - \bar{p})(p_k - \bar{p})^T$
2. 特征值分解得到局部法向量（最小特征值对应特征向量）
3. 计算夹角：$\theta_j = \arccos(|\mathbf{n}_{local} \cdot \mathbf{n}_{body}|)$
4. 过滤：保留 $\theta_j < \theta_{thr}$ 的点
5. 空间连通性聚类（BFS），保留最大连通域

**使用示例:**
```matlab
params = struct('r_local', 0.2, 'theta_thr', 30, 'cluster_dist', 0.3);
pins_refined = fig5_11_3(pins_bottom, body_surface, params);
```

---

### 步骤4: 平面拟合与共面度计算

**输入:**
- `pins_bottom_refined` - 步骤3输出的精化点云
- `params` - 参数结构体：
  - `max_iter` (default=5) - Tukey迭代次数
  - `kappa` (default=1.5) - IQR异常值系数

**输出:**
- `coplanarity_result` - 结构体包含：
  - `pins[N]` - 各引脚信息（vertex, plane_params, deviation, rmse）
  - `reference_plane[3]` - 基准平面参数 [A, B, C]
  - `coplanarity_value` - 共面度特征值（mm）
  - `max_deviation` / `min_deviation` - 极值偏差
  - `statistics_by_side` - 各侧统计信息

**核心算法:**
1. **各引脚平面拟合**（PCA方法）：
   - 协方差矩阵特征值分解
   - Tukey迭代剔除异常点：$T_{outlier} = Q_3 + \kappa \cdot IQR$
   - 提取引脚底面顶点（Z最小点）

2. **基准平面拟合**（最小二乘）：
   - 所有顶点拟合平面：$Z = A_{ref}X + B_{ref}Y + C_{ref}$

3. **共面度计算**：
   - 有向距离：$d_i = \frac{A X_i + B Y_i - Z_i + C}{\sqrt{A^2 + B^2 + 1}}$
   - 共面度：$C = \max(d_i) - \min(d_i)$

**使用示例:**
```matlab
params = struct('max_iter', 5, 'kappa', 1.5);
result = fig5_11_4(pins_refined, params);
fprintf('共面度: %.4f mm\n', result.coplanarity_value);
```

## 参数调优指南

### 高度分层参数 (步骤2)

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|----------|
| `k_z` | 2.0 | 阈值系数 | 引脚根部与底部分离明显→增大(2.5-3.0)<br>分离不明显→减小(1.5-2.0) |
| `n_bins` | 50 | 直方图分组数 | 点云密集→增大(70-100)<br>点云稀疏→减小(30-50) |

### 法向量约束参数 (步骤3)

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|----------|
| `r_local` | 0.2mm | 局部邻域半径 | 点云密集→减小(0.1-0.15)<br>点云稀疏→增大(0.25-0.3) |
| `theta_thr` | 30° | 夹角阈值 | 本体翘曲严重→增大(35-45°)<br>本体平整→减小(25-30°) |
| `cluster_dist` | 0.3mm | 聚类距离 | 接触区连续→减小(0.2-0.25)<br>接触区分散→增大(0.35-0.4) |

### 平面拟合参数 (步骤4)

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|----------|
| `max_iter` | 5 | 迭代次数 | 异常点多→增大(7-10)<br>点云质量好→减小(3-5) |
| `kappa` | 1.5 | IQR系数 | 严格剔除→减小(1.0-1.2)<br>保守剔除→增大(1.8-2.0) |

## 输出结果说明

### 1. MAT数据文件

`results/qfp_coplanarity_results.mat` 包含所有中间变量和最终结果，可用于后续分析。

**主要变量:**
- `point_cloud` - 原始点云
- `body_surface` - 步骤1输出
- `pins_bottom_candidates` - 步骤2输出
- `pins_bottom_refined` - 步骤3输出
- `coplanarity_result` - 步骤4最终结果

### 2. 测量报告

`results/coplanarity_report.txt` 包含：
- 封装本体信息（法向量、拟合误差）
- 引脚测量统计（各侧引脚数量）
- 共面度测量结果（特征值、极值偏差、标准差）
- 各侧偏差统计（均值、标准差、范围）
- 各引脚详细信息表格

### 3. 可视化图像

- `coplanarity_measurement_overview.png` - 6子图综合结果展示
- `fig5_11_1_*.png` - 步骤1本体曲面拟合可视化
- `fig5_11_2_*.png` - 步骤2高度分层可视化
- `fig5_11_3_*.png` - 步骤3法向量精化可视化
- `fig5_11_4_*.png` - 步骤4平面拟合可视化
- `fig5_11_5_*.png` - 步骤4共面度结果可视化

## 依赖关系

### 上游模块
本模块依赖于引脚定位模块（`segment/`）的输出：
- `mask_body` - 封装本体掩膜
- `pin_regions` - 引脚区域信息

可从 `segment/results/qfp_pin_localization_results.mat` 加载这些数据。

### 下游应用
共面度测量结果可用于：
- 质量检测与分级
- 生产工艺反馈
- 缺陷分析统计
- 数据库记录存档

## 故障排除

### 常见问题

**Q1: 步骤2高度分层提取的底部点云过少**
- **原因**: 阈值系数`k_z`过小
- **解决**: 增大`k_z`至2.5-3.0，或检查引脚点云质量

**Q2: 步骤3精化后点云被过度滤除**
- **原因**: `theta_thr`夹角阈值过严格
- **解决**: 增大`theta_thr`至35-40度，或检查本体曲面拟合精度

**Q3: 步骤4共面度计算结果异常（过大或过小）**
- **原因**: 
  - 基准平面拟合顶点数过少
  - 平面拟合过程异常点未充分剔除
- **解决**: 
  - 检查步骤3输出的精化点云质量
  - 调整步骤4的`kappa`参数控制异常点剔除强度

**Q4: 可视化图像显示异常或不清晰**
- **原因**: 点云尺度或视角不合适
- **解决**: 修改各`fig5_11_X.m`中`visualize_*`函数的`xlim/ylim/zlim`和`view()`参数

**Q5: 某些引脚无法测量（在结果中缺失）**
- **原因**: 
  - 引脚点云质量差（噪声大、遮挡严重）
  - 参数设置导致在中间步骤被过滤
- **解决**:
  - 检查原始点云质量
  - 放宽步骤2和步骤3的过滤参数
  - 单独运行单元测试脚本定位问题步骤

## 性能优化

### 计算效率
- **并行化**: MATLAB内置向量化已充分优化，无需额外并行
- **大数据**: 若点云超1000万点，可考虑降采样或分块处理
- **内存**: 典型2K×2K点云约需300MB内存

### 精度提升
- 使用高精度相机和投影设备
- 增加相位解包频率提高Z向分辨率
- 多次测量取平均减少随机误差
- 标定过程使用更高阶模型

## 联系与引用

**论文出处**: 第五章 5.3.2节 "扁平封装芯片引脚共面度测量"

**代码作者**: [Your Name]

**更新日期**: 2025-01-19

**许可协议**: 仅用于学术研究

---

## 更新日志

### v1.0 (2025-01-19)
- 初始版本发布
- 实现4步模块化算法
- 添加单元测试和端到端测试
- 完整可视化与报告生成
