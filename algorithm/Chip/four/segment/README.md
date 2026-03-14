# QFP芯片引脚定位算法 - 文件说明

## 📁 文件结构

### 函数文件（可被其他脚本调用）
- **fig5_10_1.m** - 粗定位函数
- **fig5_10_2.m** - 边界精化函数
- **fig5_10_3.m** - 搜索区域划分函数
- **fig5_10_4.m** - 几何约束筛选函数
- **fig5_10_5.m** - 投影分割与编号函数

### 测试脚本（可直接运行）
- **test_fig5_10_1.m** - 测试粗定位
- **test_fig5_10_2.m** - 测试边界精化
- **test_fig5_10_3.m** - 测试搜索区域划分
- **test_fig5_10_4.m** - 测试几何约束筛选
- **test_fig5_10_5.m** - 测试投影分割与编号
- **test_qfp_all.m** - 完整流程测试（推荐）

## 🚀 使用方法

### 方法1：测试单个步骤
在MATLAB中打开任意测试脚本，直接运行：
```matlab
% 例如测试第一步
test_fig5_10_1
```

### 方法2：测试完整流程（推荐）
运行完整测试脚本：
```matlab
test_qfp_all
```

### 方法3：在代码中调用函数
```matlab
% 读取图像
I = imread('fourchip.png');
I_gray = rgb2gray(I);

% 调用各个函数
[I_chip, mask_chip] = fig5_10_1(I_gray);
[body_boundary, body_mask] = fig5_10_2(I_gray, mask_chip);
search_regions = fig5_10_3(I_gray, body_boundary, mask_chip);
pins_regions = fig5_10_4(I_gray, search_regions, body_boundary);
[pins_individual, pins_masks] = fig5_10_5(I_gray, pins_regions);
```

## ✅ 优点

1. **符合MATLAB规范** - 函数定义不与文件名冲突
2. **便于调试** - 每个步骤都可以独立测试
3. **易于复用** - 函数可以被其他项目调用
4. **代码清晰** - 测试代码和函数定义分离

## 📊 输出结果

运行 `test_qfp_all.m` 后，会在 `results/` 文件夹中生成：
- `qfp_pin_localization_results.mat` - 所有中间变量
- `pin_info.csv` - 引脚信息表（ID、位置、侧面）
- `qfp_pin_localization_overview.png` - 完整流程可视化图

## 🔧 文件修改说明

### 已删除
- 旧的注释代码块（已迁移到独立测试脚本）

### 新增
- 6个独立测试脚本（test_*.m）
- 本说明文档（README.md）

### 修改
- 所有函数文件改为纯函数定义形式
- 添加了详细的函数说明注释

## 📝 注意事项

1. 确保图像路径正确：`fourchip.png`
2. 所有函数文件必须在MATLAB路径中
3. 建议先运行 `test_qfp_all.m` 查看完整效果
4. 如需调整参数，直接修改对应的函数文件

---
更新日期：2026-02-13
