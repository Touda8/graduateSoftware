---
applyTo: "test/**"
---

# 测试智能体指令

> 适用范围：`test/`（单元测试 + 集成测试）

## 测试目录结构

```
test/
├── unit/
│   ├── TestPhaseDecoder.cpp
│   ├── TestHeightCalculator.cpp
│   ├── TestBGADetector.cpp
│   ├── TestPointCloudFilter.cpp
│   └── TestPointCloudIO.cpp
├── integration/
│   ├── TestReconstructionPipeline.cpp
│   └── TestBGAMeasurePipeline.cpp
└── data/                    # 小型合成测试数据（< 5 MB，可提交）
    ├── synthetic_fringes/   # 合成条纹图（程序生成，不依赖真实相机）
    └── sample_cloud.ply     # 小型测试点云
```

## 命名约定

| 元素 | 规则 | 示例 |
|------|------|------|
| 测试文件 | `Test<ClassName>.cpp` | `TestPhaseDecoder.cpp` |
| 测试套件 | `TEST(<ClassName>, <描述>)` | `TEST(PhaseDecoder, calcPhaseMatrix_validInput_returnsRange)` |
| 测试数据文件 | 小写 + 下划线 | `synthetic_8step_freq1.bmp` |

## 必须覆盖的场景（每个公共函数）

1. **正常场景**：合法输入 → 验证输出值域 / 结果正确性
2. **边界场景**：最小 / 最大合法输入 → 不崩溃，结果合理
3. **异常场景**：非法输入 → 验证抛出正确的异常类型

## 测试断言规范

```cpp
// 矩阵数值比较（允许浮点误差）
EXPECT_LT(cv::norm(result - expected, cv::NORM_INF), 1e-6);

// 值域检查（相位图必须在 [-π, π]）
double minVal, maxVal;
cv::minMaxLoc(phase, &minVal, &maxVal);
EXPECT_GE(minVal, -CV_PI);
EXPECT_LE(maxVal,  CV_PI);

// 异常类型断言
EXPECT_THROW(decoder.calcPhaseMatrix(emptyImgs), PhaseDecodeException);
EXPECT_NO_THROW(filter.SOR(validCloud, 10, 1.0));

// 点云非空
EXPECT_GT(cloud->size(), 100u);

// 输出文件存在
EXPECT_TRUE(std::filesystem::exists("test/output/result.ply"));
```

## 测试覆盖要求清单

### PhaseDecoder
- [ ] 8 步合成正弦条纹 → 相位值域在 `[-π, π]`
- [ ] 空输入矩阵 → 抛出 `PhaseDecodeException`
- [ ] 三频展开 → 绝对相位单调性（合成无噪声场景）

### HeightCalculator
- [ ] 已知绝对相位 + 标定系数 → 输出高度误差 < 0.1 mm
- [ ] 深度梯度滤波 → 不连续区域被掩膜
- [ ] 输入尺寸不一致 → 抛出异常

### BGADetector
- [ ] 合成圆形图像 → 检测到指定数量的焊球
- [ ] 空图像 → 抛出 `MeasureException`
- [ ] 3D 定位 → 顶点坐标误差 < 0.05 mm

### PointCloudFilter
- [ ] SOR 滤波 → 输出点数 < 输入点数
- [ ] 空点云输入 → 不崩溃，返回空点云
- [ ] 体素降采样 → 输出点密度均匀

### ReconstructionPipeline（集成测试）
- [ ] 使用 `test/data/` 小样本图 → 生成非空点云（> 100 点）
- [ ] 标定文件缺失 → 友好错误，程序不崩溃
- [ ] 多次重建 → 结果一致（无状态残留）

## 构建规范

- 测试项目使用 **Debug** 配置编译（开启断言）
- 测试不依赖 UI 层任何代码（纯算法/纯逻辑测试）
- `test/data/` 中的文件大小限制：单文件 < 2 MB，总计 < 20 MB

## 验收报告格式（`docs/acceptance_report.md`）

```markdown
## 验收报告 YYYY-MM-DD HH:MM

| 测试 ID | 模块 | 用例描述 | 结果 | 备注 |
|---------|------|---------|------|------|
| T001 | PhaseDecoder | 正常 8 步相位解码 | ✅ PASS | |
| T002 | PhaseDecoder | 空输入异常 | ✅ PASS | |
| T003 | BGADetector | 焊球检测 | ❌ FAIL | 待修复：#42 |
```
