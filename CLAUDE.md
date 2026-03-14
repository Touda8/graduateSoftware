# graduateSoftware — Claude 全局指令

> 本文件对该仓库内所有 Claude 对话自动生效。多 Agent 并行开发流程通过 `.claude/commands/` 自定义技能触发。
> 详细规范见 `docs/global_description.md`、`docs/agent.md`、`docs/instruction.md`。

---

## 一、项目身份

| 项目     | 内容                                                    |
| -------- | ------------------------------------------------------- |
| 项目名称 | graduateSoftware                                        |
| 窗口标题 | 远心单目双光栅三维重建测量系统                          |
| 开发语言 | C++17                                                   |
| UI 框架  | Qt（Qt VS Tools 集成），主 UI 文件：`twoProjector.ui` |
| 主窗口类 | `twoProjectorClass : QMainWindow`                     |
| 开发环境 | Visual Studio 2019，工具集 v142，目标平台 x64           |

---

## 二、技术栈速查

| 库           | 目录                                           | 用途                 |
| ------------ | ---------------------------------------------- | -------------------- |
| OpenCV 4.5.5 | `F:\project\Envlib\opencv455\opencv`         | 图像处理、相位计算   |
| PCL 1.12.1   | `F:\project\Envlib\PCL1.12.1`                | 点云处理、滤波、测量 |
| VTK 9.1      | `F:\project\Envlib\PCL1.12.1\3rdParty\VTK`   | 点云可视化           |
| Eigen 3.4.0  | `F:\project\Envlib\eigen-3.4.0`              | 线性代数（纯头文件） |
| Boost 1.78   | `F:\project\Envlib\PCL1.12.1\3rdParty\Boost` | PCL 依赖             |
| Ceres        | `F:\project\Envlib\ceres`                    | 非线性优化           |

> **所有库只能调用，禁止修改、删除 `F:\project\Envlib` 内任何文件。**

---

## 三、Tab 结构（不可更改）

| Tab 索引 | 对象名        | 标题           | 状态               |
| -------- | ------------- | -------------- | ------------------ |
| 0        | `capWidget` | 图像采集       | **绝对禁区** |
| 1        | `resWidget` | 系统重建       | 重点开发           |
| 2        | `bgaWidget` | BGA 共面度测量 | 重点开发           |

---

## 四、最高优先约束（P01~P06）

| 编号          | 约束                                                  | 违反后果       |
| ------------- | ----------------------------------------------------- | -------------- |
| **P01** | 禁止修改 `capWidget` 及其所有子控件                 | 退出验收       |
| **P02** | 禁止修改 `F:\project\Envlib` 内任何文件             | 破坏共享库环境 |
| **P03** | UI 主线程不得执行耗时 > 5 ms 的操作                   | 界面卡顿       |
| **P04** | 计算结果必须通过 `Qt::QueuedConnection` 信号传回 UI | 跨线程崩溃     |
| **P05** | `ui_twoProjector.h`（uic 生成）禁止手动编辑         | 构建覆盖丢失   |
| **P06** | 超长文件（> 200 行/次）先创建框架再逐节插入           | 输出截断       |

---

## 五、目录职责

```
src/                 # 源代码（C++17）
├── common/          # 公共模块：interfaces.h, ThreadPool.h, Logger.h, Exceptions.h, Config.h
├── reconstruction/  # 重建模块：PhaseDecoder, HeightCalculator, ReconstructionPipeline, CalibLoader
├── measurement/     # 测量模块：BGADetector, BGAMeasurePipeline
├── pointcloud/      # 点云模块：PointCloudFilter, PointCloudIO, GeometryAnalyzer
└── visualization/   # 可视化：VtkWidget（QVTKOpenGLNativeWidget 封装）

algorithm/           # MATLAB 原始算法（只读参考，禁止修改）
test/                # 单元测试 + 集成测试（GoogleTest）
docs/                # 项目文档（设计文档、规范）

config/              # 运行时配置（.gitignore 忽略）
│   └── params.json  # 系统参数持久化

build/               # 编译输出目录（.gitignore 忽略）
├── Release/
│   ├── *.exe        # 可执行文件
│   ├── *.dll / .lib
│   └── obj/         # 中间编译产物（.obj, .pdb, .ilk）
└── Debug/
    ├── *.exe
    ├── *.dll / .lib
    └── obj/

logs/                # 运行日志、编译日志（.gitignore 忽略）
├── build_*.log      # MSBuild 编译日志（build.bat 自动生成）
└── app_*.log        # 应用运行日志（Logger 输出）
```

### 编译输出管理方式

| 产物类型       | 位置                    | 作用                                                     | 版本控制                       |
| -------------- | ----------------------- | -------------------------------------------------------- | ------------------------------ |
| 编译中间产物   | `build/<Config>/obj/` | MSBuild 编译链接阶段产生（`.obj`、`.ilk`、`.pdb`） | ✅ .gitignore                  |
| 最终可执行文件 | `build/<Config>/`     | `.exe`、`.dll`、`.lib`                             | ✅ .gitignore                  |
| 编译日志       | `logs/build_*.log`    | 每次编译自动生成时间戳日志                               | ✅ .gitignore                  |
| 应用运行日志   | `logs/app_*.log`      | 应用执行时由 Logger 写入                                 | ✅ .gitignore 已包含 `logs/` |
| 源代码配置     | `config/params.json`  | 用户自定义参数（运行时加载）                             | ✅ .gitignore                  |

---

## 六、MATLAB → C++ 翻译对照

| MATLAB 文件                                                 | C++ 目标                                    |
| ----------------------------------------------------------- | ------------------------------------------- |
| `algorithm/unwrap/CalculatePhaseMatrix.m`                 | `PhaseDecoder::calcPhaseMatrix()`         |
| `algorithm/unwrap/Multifrequency_phase.m`                 | `PhaseDecoder::multiFreqUnwrap()`         |
| `algorithm/unwrap/CalcFringeModulation.m`                 | `PhaseDecoder::calcFringeModulation()`    |
| `algorithm/unwrap/UNwrapPhase.m`                          | `PhaseDecoder::unwrapPhase()`             |
| `algorithm/unwrap/CalculatePlaneHeight.m`                 | `HeightCalculator::calcPlaneHeight()`     |
| `algorithm/unwrap/DepthGradientFilter.m`                  | `HeightCalculator::depthGradientFilter()` |
| `algorithm/Chip/BGA/segment/BGA_2D_BallDetection.m`       | `BGADetector::detect2DBalls()`            |
| `algorithm/Chip/BGA/pointCal/BGA_3D_VertexLocalization.m` | `BGADetector::localize3DBalls()`          |
| `algorithm/Chip/four/segment/QFP_Seg_Step*.m`             | `BGAMeasurePipeline` 各步骤               |
| `algorithm/Chip/four/pointCal/QFP_Cal_Step*.m`            | 共面度计算各步骤                            |

---

## 七、C++17 编码约定

### 语言规范

- 统一使用 C++17 标准 (`/std:c++17`)
- 优先使用 `std::optional<T>` 代替"可能无效"的返回值
- 文件系统操作一律使用 `std::filesystem`（禁用 `_mkdir`、`access` 等 POSIX API）
- 只读字符串参数使用 `std::string_view`，避免不必要拷贝
- 禁止使用裸 `new` / `delete`；堆内存统一使用 `std::unique_ptr` / `std::shared_ptr`
- 禁止使用 C 风格数组（用 `std::array` 或 `std::vector`）

### 命名规范

| 元素          | 规则                                      | 示例                                   |
| ------------- | ----------------------------------------- | -------------------------------------- |
| 类名          | PascalCase                                | `PhaseDecoder`, `BGADetector`      |
| 函数名        | camelCase                                 | `calcPhaseMatrix()`, `runAsync()`  |
| 成员变量      | 尾随下划线                                | `modThresh_`, `calibData_`         |
| 局部变量      | camelCase                                 | `absPhase`, `heightMap`            |
| 常量 / 枚举项 | UPPER_SNAKE_CASE                          | `MAX_FREQ_COUNT`, `DEFAULT_THRESH` |
| 宏            | `TP_` 前缀 + UPPER_SNAKE                | `TP_CHECK_EMPTY(mat)`                |
| 头文件守卫    | `#pragma once`（禁止 `#ifndef` 风格） | —                                     |

### 头文件规则

- 所有类必须分离 `.h` / `.cpp`（纯模板类例外）
- 头文件只暴露必要公共接口，实现细节留在 `.cpp`
- 使用前向声明减少 `#include` 层数
- 头文件中**禁止** `using namespace std`

### 资源管理（RAII）

- `cv::Mat`、`pcl::PointCloud::Ptr`、Qt 对象树均通过 RAII 管理
- 析构函数不抛出任何异常（加 `noexcept`）
- 跨线程传递数据用 `std::shared_ptr`，配合读写锁或 `std::atomic`

### 编码禁止事项

- 禁止全局非 `const` 变量
- 禁止将单个 `cv::Mat` 不加锁地共享给多个线程（传前 `.clone()`）
- 禁止魔法数字——阈值、频率、步数等均定义为 `constexpr` 常量或从配置读取
- 禁止注释掉大段代码提交；需保留历史用 Git 分支

### 跨线程 UI 规范

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

## 九、多智能体角色分工（Claude Agent 体系）

使用 `/skill-name` 触发对应的 Claude 自定义技能，位于 `.claude/commands/` 目录。

| 技能命令               | 定义文件                                  | 负责文件区域                                  | 开发阶段         |
| ---------------------- | ----------------------------------------- | --------------------------------------------- | ---------------- |
| `/orchestrate`       | `.claude/commands/orchestrate.md`       | 调度所有 Agent                                | 任意阶段启动入口 |
| `/analyze-needs`     | `.claude/commands/analyze-needs.md`     | `docs/`, `src/common/interfaces.h`        | Phase 1          |
| `/setup-project`     | `.claude/commands/setup-project.md`     | `*.vcxproj`, `.gitignore`, `build.bat`  | Phase 1          |
| `/build-ui`          | `.claude/commands/build-ui.md`          | `src/MainWindow.*`, `src/visualization/`  | Phase 2          |
| `/translate-algo`    | `.claude/commands/translate-algo.md`    | `src/reconstruction/`, `src/measurement/` | Phase 2          |
| `/handle-exceptions` | `.claude/commands/handle-exceptions.md` | `src/common/Exceptions.h`, `Logger.*`     | Phase 3          |
| `/run-tests`         | `.claude/commands/run-tests.md`         | `test/`                                     | Phase 3          |

> **用户直接使用 `/orchestrate` 启动完整并行构建流程，无需手动切换各子技能。**

### Claude Agent 工具映射

| 原 Copilot 工具                        | Claude 工具替代                                        |
| -------------------------------------- | ------------------------------------------------------ |
| `runSubagent`                        | `Agent` tool（`subagent_type: "general-purpose"`） |
| `editFiles`                          | `Edit` / `Write` tool                              |
| `runCommands`                        | `Bash` tool                                          |
| `codebase` / `search` / `usages` | `Glob` / `Grep` / `Read` tool                    |
| `create_file`                        | `Write` tool                                         |
| `replace_string_in_file`             | `Edit` tool                                          |
| `read_file`                          | `Read` tool                                          |
| `problems`                           | `ToolSearch` + `Bash` (编译诊断)                   |

---

