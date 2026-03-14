# 开发任务清单

> 按 Phase 分阶段、按优先级排序的所有开发任务。

## Phase 1 — 需求解析 + 工程配置

| 编号 | 优先级 | 模块 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|------|---------|-----------|------|------|
| T001 | P0 | docs | 生成 `docs/forbidden_list.md` 禁区控件清单 | Agent1（需求解析） | twoProjector.ui | ☐ |
| T002 | P0 | docs | 生成 `docs/ui_algorithm_binding.md` UI-算法绑定表 | Agent1（需求解析） | T001 | ☐ |
| T003 | P0 | docs | 生成 `docs/task_list.md` 开发任务清单 | Agent1（需求解析） | T001, T002 | ☐ |
| T004 | P0 | common | 生成 `src/common/interfaces.h` 公共接口定义 | Agent1（需求解析） | T002 | ☐ |
| T005 | P0 | build | 创建 VS2019 解决方案 `twoProjector.sln` + `.vcxproj` | Agent4（工程配置） | — | ☐ |
| T006 | P0 | build | 配置 `F:\project\Envlib` 所有库路径属性表 `.vcxproj.props` | Agent4（工程配置） | T005 | ☐ |
| T007 | P0 | build | 配置 Qt VS Tools 属性、uic/moc 构建规则 | Agent4（工程配置） | T005 | ☐ |
| T008 | P0 | build | 生成 `.gitignore` + `build.bat` + Git 初始化 | Agent4（工程配置） | T005 | ☐ |

## Phase 2 — UI 绑定 + 算法翻译

### 模块：PhaseDecoder

| 编号 | 优先级 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|---------|-----------|------|------|
| T101 | P1 | 实现 `PhaseDecoder::calcPhaseMatrix(imgs, N)` — N步相移主值相位计算 | Agent3（算法） | T004 | ☐ |
| T102 | P1 | 实现 `PhaseDecoder::multiFreqUnwrap(phi1, phi2, phi3, f1, f2, f3)` — 三频绝对相位展开 | Agent3（算法） | T101 | ☐ |
| T103 | P1 | 实现 `PhaseDecoder::calcFringeModulation(imgs, N)` — 条纹调制度计算 | Agent3（算法） | T004 | ☐ |
| T104 | P1 | 实现 `PhaseDecoder::calcModulationContrast(imgs, N)` — 调制度对比度计算 | Agent3（算法） | T103 | ☐ |
| T105 | P1 | 实现 `PhaseDecoder::calcPhaseErrorEnergy(imgs, phase, N)` — 相位误差能量计算 | Agent3（算法） | T103 | ☐ |
| T106 | P1 | 实现 `PhaseDecoder::unwrapPhase(keyPhase, multiple, wrappedPhase)` — 时间相位展开 | Agent3（算法） | T101 | ☐ |

### 模块：HeightCalculator

| 编号 | 优先级 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|---------|-----------|------|------|
| T201 | P1 | 实现 `HeightCalculator::calcPlaneHeight(absPhase, calib)` — 相位→高度映射 | Agent3（算法） | T004 | ☐ |
| T202 | P1 | 实现 `HeightCalculator::depthGradientFilter(heightMap, mask, gradThresh)` — 深度梯度滤波 | Agent3（算法） | T201 | ☐ |
| T203 | P1 | 实现 `HeightCalculator::phaseQualityMask(...)` — 多特征融合质量掩膜 | Agent3（算法） | T101, T103 | ☐ |
| T204 | P1 | 实现 `HeightCalculator::generateMask(imgs, N)` — 调制度掩膜生成 | Agent3（算法） | T103 | ☐ |

### 模块：PointCloudFilter

| 编号 | 优先级 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|---------|-----------|------|------|
| T301 | P2 | 实现 `PointCloudFilter::SOR(cloud, k, stdMul)` — 统计离群值滤波 | Agent3（算法） | T004 | ☐ |
| T302 | P2 | 实现 `PointCloudFilter::ROR(cloud, radius, minN)` — 半径离群值滤波 | Agent3（算法） | T004 | ☐ |
| T303 | P2 | 实现 `PointCloudFilter::passThrough(cloud, axis, min, max)` — 直通滤波 | Agent3（算法） | T004 | ☐ |
| T304 | P2 | 实现 `PointCloudFilter::voxelGrid(cloud, leafSize)` — 体素降采样 | Agent3（算法） | T004 | ☐ |

### 模块：GeometryAnalyzer

| 编号 | 优先级 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|---------|-----------|------|------|
| T401 | P2 | 实现 `GeometryAnalyzer::fitPlanePCA(cloud)` — PCA 平面拟合 | Agent3（算法） | T004 | ☐ |
| T402 | P2 | 实现 `GeometryAnalyzer::fitPlaneLeastSquares(cloud)` — 最小二乘平面拟合 | Agent3（算法） | T004 | ☐ |
| T403 | P2 | 实现 `GeometryAnalyzer::calcRoughness(cloud)` — 粗糙度估计 | Agent3（算法） | T004 | ☐ |
| T404 | P2 | 实现 `GeometryAnalyzer::distanceToPlane(cloud, plane)` — 点到平面距离映射 | Agent3（算法） | T401 | ☐ |

### 模块：BGADetector

| 编号 | 优先级 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|---------|-----------|------|------|
| T501 | P1 | 实现 `BGADetector::detect2DBalls(brightImg, mask)` — BGA 焊球二维检测 | Agent3（算法） | T004 | ☐ |
| T502 | P1 | 实现 `BGADetector::localize3DBalls(cloud, balls2D, calib)` — BGA 焊球三维定位 | Agent3（算法） | T501 | ☐ |
| T503 | P2 | 实现 `BGAMeasurePipeline::runAsync()` — BGA 共面度测量主流程 | Agent3（算法） | T501, T502 | ☐ |

### 模块：QFPSegmentor + QFPCalculator

| 编号 | 优先级 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|---------|-----------|------|------|
| T601 | P2 | 实现 `QFPSegmentor::step1ChipLocalization(img)` — 芯片粗定位 | Agent3（算法） | T004 | ☐ |
| T602 | P2 | 实现 `QFPSegmentor::step2BodyBoundary(img, mask)` — 封装本体边界精化 | Agent3（算法） | T601 | ☐ |
| T603 | P2 | 实现 `QFPSegmentor::step3SearchRegion(img, mask, chipInfo)` — 引脚搜索区划分 | Agent3（算法） | T602 | ☐ |
| T604 | P2 | 实现 `QFPSegmentor::step4GeometricFilter(img, regions, boundary)` — 多约束几何滤波 | Agent3（算法） | T603 | ☐ |
| T605 | P2 | 实现 `QFPSegmentor::step5PinSegmentation(img, regions)` — 单引脚投影分割 | Agent3（算法） | T604 | ☐ |
| T606 | P2 | 实现 `QFPCalculator::step1BodySurfaceFit(X, Y, Z, mask)` — 本体曲面拟合 | Agent3（算法） | T004 | ☐ |
| T607 | P2 | 实现 `QFPCalculator::step2PinTipExtraction(X, Y, Z, regions, params)` — 引脚底部提取 | Agent3（算法） | T606 | ☐ |
| T608 | P2 | 实现 `QFPCalculator::step3PointCloudRefine(candidates, surface, params)` — 法向量约束精化 | Agent3（算法） | T607 | ☐ |
| T609 | P2 | 实现 `QFPCalculator::step4CoplanarityCalc(refined, params)` — 共面度与翘曲度计算 | Agent3（算法） | T608 | ☐ |

### 模块：ReconstructionPipeline + CalibLoader + PointCloudIO

| 编号 | 优先级 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|---------|-----------|------|------|
| T701 | P1 | 实现 `CalibLoader::load(path)` — 标定文件解析（.raw/.mat） | Agent3（算法） | T004 | ☐ |
| T702 | P1 | 实现 `ReconstructionPipeline::runAsync(scanFolder)` — 重建主流水线 | Agent3（算法） | T101~T106, T201~T204, T701 | ☐ |
| T703 | P2 | 实现 `PointCloudIO::load(path)` — PLY/PCD/OBJ/STL 读取 | Agent3（算法） | T004 | ☐ |
| T704 | P2 | 实现 `PointCloudIO::save(cloud, path, format)` — PLY/PCD/CSV 写入 | Agent3（算法） | T004 | ☐ |
| T705 | P2 | 实现 `PointCloudMerger::merge(cloud1, cloud2)` — Pro1+Pro2 点云合并 | Agent3（算法） | T004 | ☐ |

### 模块：MainWindow + VtkWidget + Config

| 编号 | 优先级 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|---------|-----------|------|------|
| T801 | P1 | 实现 `MainWindow` 类骨架 — 构造函数、成员声明 | Agent2（UI） | T004, T002 | ☐ |
| T802 | P1 | 绑定 resWidget 所有信号槽（文件选择、重建控制） | Agent2（UI） | T801, T002 | ☐ |
| T803 | P1 | 绑定 bgaWidget 所有信号槽 | Agent2（UI） | T801, T002 | ☐ |
| T804 | P1 | 绑定 qfpWidget 所有信号槽 | Agent2（UI） | T801, T002 | ☐ |
| T805 | P1 | 绑定点云处理区所有信号槽 | Agent2（UI） | T801, T002 | ☐ |
| T806 | P2 | 实现 `VtkWidget` — QVTKOpenGLNativeWidget 封装 | Agent2（UI） | T801 | ☐ |
| T807 | P2 | 实现 `Config::loadFromJson()` / `saveToJson()` — 参数持久化 | Agent2（UI） | T801 | ☐ |
| T808 | P2 | 实现重建状态机（IDLE→CALIB_LOADING→READY→RECONSTRUCTING→DONE） | Agent2（UI） | T802 | ☐ |
| T809 | P2 | 实现 resProgressBar 进度回调（QueuedConnection） | Agent2（UI） | T802 | ☐ |
| T810 | P2 | 实现 resLogText 日志追加（线程安全） | Agent2（UI） | T802 | ☐ |

## Phase 3 — 异常处理 + 测试

| 编号 | 优先级 | 模块 | 任务描述 | 负责 Agent | 依赖 | 状态 |
|------|-------|------|---------|-----------|------|------|
| T901 | P2 | common | 实现 `Exceptions.h` — 完整异常类层次 | Agent5（异常） | T004 | ☐ |
| T902 | P2 | common | 实现 `Logger.h/.cpp` — 线程安全滚动日志 | Agent5（异常） | T004 | ☐ |
| T903 | P2 | common | 为所有算法函数添加异常捕获点 | Agent5（异常） | T901, T101~T609 | ☐ |
| T904 | P2 | common | 为 UI 槽函数添加全局 try/catch | Agent5（异常） | T901, T801~T810 | ☐ |
| T905 | P2 | common | 生成 `docs/exception_scenarios.md` 异常场景清单 | Agent5（异常） | T903 | ☐ |
| T906 | P3 | test | 编写 `test/test_phase.cpp` — PhaseDecoder 单元测试 | Agent6（测试） | T101~T106 | ☐ |
| T907 | P3 | test | 编写 `test/test_pointcloud.cpp` — 点云处理单元测试 | Agent6（测试） | T301~T304 | ☐ |
| T908 | P3 | test | 编写 `test/test_bga.cpp` — BGA 检测单元测试 | Agent6（测试） | T501~T502 | ☐ |
| T909 | P3 | test | 编写 `test/test_exception.cpp` — 异常捕获测试 | Agent6（测试） | T901~T904 | ☐ |
| T910 | P3 | test | 一键编译验证（Release x64 零错误零警告） | Agent6（测试） | 全部 | ☐ |
| T911 | P3 | test | 生成 `docs/acceptance_report.md` 验收报告 | Agent6（测试） | T910 | ☐ |
