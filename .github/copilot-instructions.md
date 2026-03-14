# graduateSoftware — Copilot 全局指令

> 本文件对该仓库内所有 Copilot 对话自动生效。详细规范见 `docs/global_description.md`、`docs/agent.md`、`docs/instruction.md`。

---

## 一、项目身份

| 项目 | 内容 |
|------|------|
| 项目名称 | graduateSoftware |
| 窗口标题 | 远心单目双光栅三维重建测量系统 |
| 开发语言 | C++17 |
| UI 框架 | Qt（Qt VS Tools 集成），主 UI 文件：`twoProjector.ui` |
| 主窗口类 | `twoProjectorClass : QMainWindow` |
| 开发环境 | Visual Studio 2019，工具集 v142，目标平台 x64 |

---

## 二、技术栈速查

| 库 | 目录 | 用途 |
|----|------|------|
| OpenCV 4.5.5 | `F:\project\Envlib\opencv455\opencv` | 图像处理、相位计算 |
| PCL 1.12.1 | `F:\project\Envlib\PCL1.12.1` | 点云处理、滤波、测量 |
| VTK 9.1 | `F:\project\Envlib\PCL1.12.1\3rdParty\VTK` | 点云可视化 |
| Eigen 3.4.0 | `F:\project\Envlib\eigen-3.4.0` | 线性代数（纯头文件） |
| Boost 1.78 | `F:\project\Envlib\PCL1.12.1\3rdParty\Boost` | PCL 依赖 |
| Ceres | `F:\project\Envlib\ceres` | 非线性优化 |

> **所有库只能调用，禁止修改、删除 `F:\project\Envlib` 内任何文件。**

---

## 三、Tab 结构（不可更改）

| Tab 索引 | 对象名 | 标题 | 状态 |
|----------|--------|------|------|
| 0 | `capWidget` | 图像采集 | **绝对禁区** |
| 1 | `resWidget` | 系统重建 | 重点开发 |
| 2 | `bgaWidget` | BGA 共面度测量 | 重点开发 |

---

## 四、最高优先约束（P01~P06）

| 编号 | 约束 | 违反后果 |
|------|------|---------|
| **P01** | 禁止修改 `capWidget` 及其所有子控件 | 退出验收 |
| **P02** | 禁止修改 `F:\project\Envlib` 内任何文件 | 破坏共享库环境 |
| **P03** | UI 主线程不得执行耗时 > 5 ms 的操作 | 界面卡顿 |
| **P04** | 计算结果必须通过 `Qt::QueuedConnection` 信号传回 UI | 跨线程崩溃 |
| **P05** | `ui_twoProjector.h`（uic 生成）禁止手动编辑 | 构建覆盖丢失 |
| **P06** | 超长文件（> 5000 字符）先创建框架再逐节插入 | 输出截断 |

---

## 五、目录职责

```
src/
├── common/          # interfaces.h, ThreadPool.h, Logger.h, Exceptions.h, Config.h
├── reconstruction/  # PhaseDecoder, HeightCalculator, ReconstructionPipeline, CalibLoader
├── measurement/     # BGADetector, BGAMeasurePipeline
├── pointcloud/      # PointCloudFilter, PointCloudIO, GeometryAnalyzer
└── visualization/   # VtkWidget（QVTKOpenGLNativeWidget 封装）
algorithm/           # MATLAB 原始算法（只读参考，禁止修改）
config/              # params.json（运行时参数持久化）
logs/                # 运行日志（.gitignore 忽略）
test/                # 单元测试 + 集成测试
docs/                # 项目文档
```

---

## 六、MATLAB → C++ 翻译对照

| MATLAB 文件 | C++ 目标 |
|------------|---------|
| `algorithm/unwrap/CalculatePhaseMatrix.m` | `PhaseDecoder::calcPhaseMatrix()` |
| `algorithm/unwrap/Multifrequency_phase.m` | `PhaseDecoder::multiFreqUnwrap()` |
| `algorithm/unwrap/CalcFringeModulation.m` | `PhaseDecoder::calcFringeModulation()` |
| `algorithm/unwrap/UNwrapPhase.m` | `PhaseDecoder::unwrapPhase()` |
| `algorithm/unwrap/CalculatePlaneHeight.m` | `HeightCalculator::calcPlaneHeight()` |
| `algorithm/unwrap/DepthGradientFilter.m` | `HeightCalculator::depthGradientFilter()` |
| `algorithm/Chip/BGA/segment/BGA_2D_BallDetection.m` | `BGADetector::detect2DBalls()` |
| `algorithm/Chip/BGA/pointCal/BGA_3D_VertexLocalization.m` | `BGADetector::localize3DBalls()` |
| `algorithm/Chip/four/segment/QFP_Seg_Step*.m` | `BGAMeasurePipeline` 各步骤 |
| `algorithm/Chip/four/pointCal/QFP_Cal_Step*.m` | 共面度计算各步骤 |

---

## 七、编码约定速查

```cpp
// ✅ 正确：跨线程更新 UI
connect(worker, &Worker::progressUpdated,
        ui->resProgressBar, &QProgressBar::setValue,
        Qt::QueuedConnection);

// ❌ 禁止：在工作线程直接操作控件
ui->resProgressBar->setValue(50);

// ✅ 控件命名前缀规则
// res* → 系统重建 Tab
// pc*  → 点云处理（resWidget 子区域）
// bga* → BGA 测量 Tab
// cap* → 图像采集 Tab（禁区，不得出现在新代码中）
```

---

## 八、Git 提交格式

```
<type>(<scope>): <subject>

类型: feat | fix | refactor | test | docs | build | chore
作用域: ui | algo | pcl | build | test | common | meas

示例: feat(algo): implement PhaseDecoder::multiFreqUnwrap
```

---

## 九、多智能体角色分工

在 VS Code Copilot Chat 的 **Agent 下拉菜单**中选择对应角色后发起对话。
详细操作流程见项目根目录 `开发操作指南.md`。

| 显示名称 | 定义文件 | 负责文件区域 | 开发阶段 |
|---------|---------|------------|---------|
| 总指挥 (Chief Orchestrator) | `.github/agents/chief-orchestrator.md` | 调度所有 Agent | 任意阶段启动入口 |
| 需求解析师 (Needs Analyst) | `.github/agents/needs-analyst.md` | `docs/`, `src/common/interfaces.h` | Phase 1 |
| 工程配置师 (Project Engineer) | `.github/agents/project-engineer.md` | `*.vcxproj`, `.gitignore`, `build.bat` | Phase 1 |
| UI 开发师 (UI Builder) | `.github/agents/ui-builder.md` | `src/MainWindow.*`, `src/visualization/` | Phase 2 |
| 算法翻译师 (Algo Translator) | `.github/agents/algo-translator.md` | `src/reconstruction/`, `src/measurement/` | Phase 2 |
| 异常防护师 (Exception Handler) | `.github/agents/exception-handler.md` | `src/common/Exceptions.h`, `Logger.*` | Phase 3 |
| 测试验收师 (Test Runner) | `.github/agents/test-runner.md` | `test/` | Phase 3 |
