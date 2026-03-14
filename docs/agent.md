# agent.md — 多智能体角色定义

## 概览：多智能体协作流程

```
Agent1（需求解析）
    ↓ 输出：任务清单 + UI-算法绑定清单 + 禁止修改清单
    ├──────────────────────────────────────────────────────
    ↓               ↓               ↓              ↓
Agent2（UI）  Agent3（算法）  Agent4（工程）  Agent5（异常）
    ↓               ↓               ↓              ↓
    └───────────────────── → Agent6（测试验收）
```

- Agent1 率先完成，输出物供其余 Agent 并行启动；
- Agent2、Agent3、Agent4、Agent5 可**并行**开发，通过接口文档（`src/common/` 头文件）解耦；
- Agent6 在其余 Agent 完成后执行集成测试与验收。

---

## Agent 1：需求解析智能体

### 职责
1. 解析 `global_description.md` 与 `twoProjector.ui`，生成函数级任务清单；
2. 输出 UI 控件 → 算法函数的完整绑定表（含信号、槽函数签名）；
3. 输出禁止修改清单、性能指标、模块接口约定（`.h` 头文件规范）。

### 输入
- `docs/global_description.md`
- `twoProjector.ui`（全文解析）
- `algorithm/函数功能说明.md`

### 输出物（必须生成）

| 输出文件 | 内容 |
|---------|------|
| `docs/task_list.md` | 按模块细化的任务清单，粒度到函数签名 |
| `docs/ui_algorithm_binding.md` | 每个可操作控件 → 对应槽函数 → 调用算法函数的三级映射 |
| `docs/forbidden_list.md` | 禁止修改的控件/文件/目录清单，附理由 |
| `src/common/interfaces.h` | 各模块公共接口头文件（纯虚基类或结构体定义） |

### 关键约束分析输出

```markdown
# 禁止修改清单（示例格式）
- twoProjector.ui 中的 capWidget 及其所有子控件（图像采集 Tab）
- F:\project\Envlib\ 下所有文件
- ui_twoProjector.h（uic 自动生成，每次构建覆盖）
```

### Task 清单格式（示例）

```markdown
## 模块：PhaseDecoder
- [ ] T001 实现 `PhaseDecoder::calcPhaseMatrix(imgs, N)` — 八步相移主值相位计算
- [ ] T002 实现 `PhaseDecoder::multiFreqUnwrap(phi1, phi2, phi3, f1, f2, f3)` — 三频绝对相位
- [ ] T003 实现 `PhaseDecoder::calcFringeModulation(imgs, N)` — 调制度图计算
```

---

## Agent 2：UI 开发智能体

### 职责
1. 基于 `twoProjector.ui`（Qt uic 生成 `ui_twoProjector.h`），用 C++ 实现 `MainWindow` 类；
2. 仅开发 `resWidget`（系统重建）和 `bgaWidget`（BGA 共面度测量）两个 Tab；
3. 按 `ui_algorithm_binding.md` 绑定所有信号槽；
4. 实现参数保存/加载（JSON <=> UI 控件双向同步）；
5. 实现 UI 与计算线程完全分离，保证响应延迟 < 200 ms；
6. 对接 VTK 渲染到 `resCloudLabel`（替换为 `QVTKOpenGLNativeWidget`）。

### 输入
- `ui_algorithm_binding.md`（Agent1 输出）
- `src/common/interfaces.h`（Agent1 输出）
- `twoProjector.ui`

### 禁止操作
- 不修改任何属于 `capWidget` 的控件；
- 不直接调用算法实现代码（通过接口调用）；
- 不在 UI 线程执行耗时操作（> 5 ms 的逻辑移至工作线程）。

### 输出物

| 文件 | 说明 |
|------|------|
| `src/MainWindow.h` | 主窗口头文件，声明槽函数、成员变量 |
| `src/MainWindow.cpp` | 信号槽绑定、UI 更新逻辑 |
| `src/visualization/VtkWidget.h/.cpp` | VTK 嵌入 Qt 封装，接管 `resCloudLabel` |
| `src/common/Config.h/.cpp` | JSON 参数读写，支持 UI 控件↔JSON 双向同步 |
| `docs/ui_logic.md` | 界面逻辑文档：每个按钮的执行流程（含状态机说明） |

### 关键信号槽清单

```cpp
// 系统重建 Tab
connect(resBtnLoadCalib,    &QPushButton::clicked, this, &MainWindow::onLoadCalib);
connect(resBtnStartRebuild, &QPushButton::clicked, this, &MainWindow::onStartRebuild);
connect(resBtnSaveCloud,    &QPushButton::clicked, this, &MainWindow::onSaveCloud);
connect(resBtnTogglePanel,  &QPushButton::clicked, this, &MainWindow::onTogglePanel);
connect(resComboSaveMode,   QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, &MainWindow::onSaveModeChanged);

// 点云处理
connect(pcBtnImportCloud,   &QPushButton::clicked, this, &MainWindow::onImportCloud);
connect(pcBtnSOR,           &QPushButton::clicked, this, &MainWindow::onSORFilter);
// ... 其余点云操作按钮类似

// BGA 测量 Tab
connect(bgaBtnStart,        &QPushButton::clicked, this, &MainWindow::onBGAStart);
connect(bgaBtnPlot,         &QPushButton::clicked, this, &MainWindow::onBGAPlot);
connect(bgaBtnSave,         &QPushButton::clicked, this, &MainWindow::onBGASave);

// 计算线程回调（跨线程，QueuedConnection）
connect(this, &MainWindow::progressUpdated, resProgressBar, &QProgressBar::setValue,
        Qt::QueuedConnection);
connect(this, &MainWindow::logMessage, this, &MainWindow::appendLog,
        Qt::QueuedConnection);
connect(this, &MainWindow::phaseImageReady, this, &MainWindow::updatePhaseDisplay,
        Qt::QueuedConnection);
```

### 状态机（系统重建流程）

```
[IDLE] 
  → 点击"加载标定文件" → [CALIB_LOADING]
      → 成功 → [READY]
      → 失败 → [IDLE] + 弹窗
  → 点击"开始重建"（需处于 READY）→ [RECONSTRUCTING]
      → 成功 → [DONE]，解锁"保存点云"
      → 失败 → [READY] + 日志提示
  → 点击"保存点云"（需处于 DONE）→ 保存文件 → [DONE]
```

---

## Agent 3：算法开发智能体

### 职责
1. 将 `algorithm/` 中 MATLAB 算法逐一翻译为 C++（见翻译对照表）；
2. 基于 OpenCV + PCL + Eigen 实现单目双投影重建与 BGA 测量；
3. 所有核心计算接入 `ThreadPool`，Pro1/Pro2 解码并行执行；
4. 提供统一的 C++ API，供 Agent2 通过接口调用。

### 输入
- `algorithm/` 所有 `.m` 文件
- `algorithm/函数功能说明.md`
- `src/common/interfaces.h`（Agent1 定义）

### 核心类实现清单

#### `PhaseDecoder`（`src/reconstruction/PhaseDecoder.h/.cpp`）

```cpp
class PhaseDecoder {
public:
    // 八步相移主值相位：imgs[freq][step]，返回相位 [-π, π]
    cv::Mat calcPhaseMatrix(const std::array<cv::Mat, 8>& imgs) const;

    // 三频绝对相位展开
    cv::Mat multiFreqUnwrap(const cv::Mat& phi1, const cv::Mat& phi2,
                            const cv::Mat& phi3,
                            int f1, int f2, int f3) const;

    // 调制度图（用于质量掩膜）
    cv::Mat calcFringeModulation(const std::array<cv::Mat, 8>& imgs) const;

    // 调制度对比度
    cv::Mat calcModulationContrast(const std::array<cv::Mat, 8>& imgs) const;

    // 相位误差能量
    cv::Mat calcPhaseErrorEnergy(const cv::Mat& phase) const;

    // 相位展开（空间域）
    cv::Mat unwrapPhase(const cv::Mat& wrappedPhase) const;
};
```

#### `HeightCalculator`（`src/reconstruction/HeightCalculator.h/.cpp`）

```cpp
struct CalibData {
    cv::Mat allK;       // 相位-高度多项式系数
    cv::Mat allK_i;     // 逆映射系数
    cv::Mat X, Y;       // 世界坐标 X/Y 映射
};

class HeightCalculator {
public:
    // 由绝对相位和标定系数计算高度图
    cv::Mat calcPlaneHeight(const cv::Mat& absPhase,
                            const CalibData& calib) const;

    // 深度梯度滤波（去除非连续点）
    cv::Mat depthGradientFilter(const cv::Mat& heightMap,
                                double gradThresh) const;

    // 相位质量掩膜
    cv::Mat phaseQualityMask(const cv::Mat& modulation,
                             double modThresh) const;
};
```

#### `ReconstructionPipeline`（`src/reconstruction/ReconstructionPipeline.h/.cpp`）

```cpp
class ReconstructionPipeline : public QObject {
    Q_OBJECT
public:
    struct Params { /* 来自 UI 控件的所有参数 */ };

    void setCalib(const CalibData& c1, const CalibData& c2);
    void setParams(const Params& p);

    // 异步启动（在 ThreadPool 中执行，不阻塞 UI）
    void runAsync(const QString& scanFolder);

signals:
    void progressUpdated(int percent);
    void logMessage(QtMsgType type, const QString& msg);
    void phaseReady(int projIndex, const QImage& phaseImg);
    void cloudReady(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void finished(bool success, const QString& errorMsg);
};
```

#### `BGADetector`（`src/measurement/BGADetector.h/.cpp`）

```cpp
class BGADetector {
public:
    // 二维焊球检测（使用全亮图 + Hough 圆检测）
    std::vector<cv::Vec3f> detect2DBalls(const cv::Mat& brightImage,
                                          const cv::Mat& mask) const;

    // 三维焊球顶点定位（在点云中提取球顶）
    std::vector<Eigen::Vector3d> localize3DBalls(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const std::vector<cv::Vec3f>& balls2D,
        const CalibData& calib) const;
};
```

#### `PointCloudFilter`（`src/pointcloud/PointCloudFilter.h/.cpp`）

```cpp
class PointCloudFilter {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr SOR(
        pcl::PointCloud<pcl::PointXYZ>::Ptr in, int k, double stdMul);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr ROR(
        pcl::PointCloud<pcl::PointXYZ>::Ptr in, double radius, int minNeighbors);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough(
        pcl::PointCloud<pcl::PointXYZ>::Ptr in,
        const std::string& axis, float min, float max);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGrid(
        pcl::PointCloud<pcl::PointXYZ>::Ptr in, float leafSize);
};
```

### 并行化策略

| 任务 | 并行方式 | 依赖关系 |
|------|---------|---------|
| 读取 Pro1 图像 | Task A，独立 | 无 |
| 读取 Pro2 图像 | Task B，与 A 并行 | 无 |
| Pro1 三频相位解码 | Task C，A 完成后 | Task A |
| Pro2 三频相位解码 | Task D，B 完成后 | Task B |
| 极线相位修正 | Task E | C + D 均完成 |
| 高度计算 + 点云生成 | Task F | E 完成 |
| 点云后处理（可选） | Task G，F 完成后并行子任务 | F |

### 输出物

| 文件 | 说明 |
|------|------|
| `src/reconstruction/PhaseDecoder.h/.cpp` | 相位解码全实现 |
| `src/reconstruction/HeightCalculator.h/.cpp` | 高度计算 |
| `src/reconstruction/ReconstructionPipeline.h/.cpp` | 重建主流水线 |
| `src/reconstruction/CalibLoader.h/.cpp` | 标定文件加载与解析 |
| `src/measurement/BGADetector.h/.cpp` | BGA 检测与定位 |
| `src/measurement/BGAMeasurePipeline.h/.cpp` | BGA 共面度测量主流程 |
| `src/pointcloud/PointCloudFilter.h/.cpp` | 滤波模块 |
| `src/pointcloud/PointCloudIO.h/.cpp` | PLY/PCD/CSV 读写 |
| `src/pointcloud/GeometryAnalyzer.h/.cpp` | 平面拟合、距离映射 |
| `src/common/ThreadPool.h` | 线程池（header-only） |

---

## Agent 4：工程化智能体

### 职责
1. 创建 VS2019 解决方案（`twoProjector.sln` + `twoProjector.vcxproj`）；
2. 自动检测并配置 `F:\project\Envlib` 中的所有库路径（**不写死版本号，通过属性表动态配置**）；
3. 添加所有 `src/` 源文件到工程；
4. 配置 Qt 集成（Qt VS Tools 属性表、uic/moc 构建规则）；
5. 配置多线程编译（`/MP`）；
6. 执行 Git 初始化，生成 `.gitignore`，推送 `main`；
7. 生成快捷构建脚本 `build.bat`。

### 库路径配置（vcxproj 属性表格式）

```xml
<!-- 以 Release|x64 为例 -->
<PropertyGroup Label="EnvLib">
  <EnvLib>F:\project\Envlib</EnvLib>
  <OpenCVDir>$(EnvLib)\opencv455\opencv</OpenCVDir>
  <PCLDir>$(EnvLib)\PCL1.12.1</PCLDir>
  <EigenDir>$(EnvLib)\eigen-3.4.0</EigenDir>
  <CeresDir>$(EnvLib)\ceres</CeresDir>
</PropertyGroup>

<ItemDefinitionGroup Condition="'$(Configuration)|$(Platform)'=='Release|x64'">
  <ClCompile>
    <AdditionalIncludeDirectories>
      $(OpenCVDir)\include;
      $(PCLDir)\include\pcl-1.12;
      $(PCLDir)\3rdParty\Boost\include\boost-1_78;
      $(PCLDir)\3rdParty\Eigen\eigen3;
      $(PCLDir)\3rdParty\FLANN\include;
      $(PCLDir)\3rdParty\VTK\include\vtk-9.1;
      $(PCLDir)\3rdParty\Qhull\include;
      $(EigenDir);
      $(CeresDir)\include;
      %(AdditionalIncludeDirectories)
    </AdditionalIncludeDirectories>
    <PreprocessorDefinitions>
      NDEBUG;_WINDOWS;NOMINMAX;_USE_MATH_DEFINES;
      %(PreprocessorDefinitions)
    </PreprocessorDefinitions>
    <RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>
    <MultiProcessorCompilation>true</MultiProcessorCompilation>
    <LanguageStandard>stdcpp17</LanguageStandard>
  </ClCompile>
  <Link>
    <AdditionalLibraryDirectories>
      $(OpenCVDir)\x64\vc16\lib;
      $(PCLDir)\lib;
      $(PCLDir)\3rdParty\Boost\lib;
      $(PCLDir)\3rdParty\FLANN\lib;
      $(PCLDir)\3rdParty\VTK\lib;
      $(PCLDir)\3rdParty\Qhull\lib;
      $(CeresDir)\lib;
      %(AdditionalLibraryDirectories)
    </AdditionalLibraryDirectories>
    <!-- 具体 .lib 列表由 Agent4 运行时检测 Envlib 目录自动输出 -->
  </Link>
</ItemDefinitionGroup>
```

### 关键编译开关

| 开关 | 值 | 原因 |
|------|-----|------|
| `/std:c++17` | 开启 | 需要 `std::optional`, `std::filesystem` |
| `/MP` | 开启 | 多处理器并行编译加速 |
| `/W3` | 开启 | 三级警告，忽略第三方库警告 |
| `/DNOMINMAX` | 开启 | 避免 Windows.h 的 min/max 宏冲突 |
| `/D_USE_MATH_DEFINES` | 开启 | 启用 `M_PI` 等数学常数 |
| `/Zi` Debug | 开启 | 调试信息 |
| `/O2` Release | 开启 | 最优化 |

### 输出物

| 文件 | 说明 |
|------|------|
| `twoProjector.sln` | VS2019 解决方案 |
| `twoProjector.vcxproj` | 工程文件（含所有配置） |
| `twoProjector.vcxproj.props` | 库路径属性表（可复用） |
| `.gitignore` | 完整忽略规则 |
| `build.bat` | 一键调用 MSBuild 的批处理脚本 |
| `docs/build_guide.md` | 编译说明文档 |

### build.bat 参考

```bat
@echo off
set VS_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019
call "%VS_PATH%\Common7\Tools\VsDevCmd.bat" -arch=x64
msbuild twoProjector.sln /p:Configuration=Release /p:Platform=x64 /m
pause
```

---

## Agent 5：异常分析智能体

### 职责
1. 实现 `src/common/Exceptions.h`（完整异常类层次）；
2. 实现 `src/common/Logger.h/.cpp`（线程安全日志，滚动文件）；
3. 为 Agent3 的所有算法函数添加异常捕获点；
4. 为 Agent2 的 UI 槽函数添加全局 try/catch；
5. 输出异常场景清单与测试用例供 Agent6 使用。

### 异常类实现规范

```cpp
// src/common/Exceptions.h
class AppException : public std::exception {
public:
    AppException(const std::string& module, const std::string& msg)
        : module_(module), msg_(msg) {}
    const char* what() const noexcept override { return msg_.c_str(); }
    const std::string& module() const { return module_; }
private:
    std::string module_, msg_;
};

#define DEFINE_EXCEPTION(Name) \
    class Name : public AppException { \
    public: using AppException::AppException; };

DEFINE_EXCEPTION(CalibException)
DEFINE_EXCEPTION(ImageLoadException)
DEFINE_EXCEPTION(PhaseDecodeException)
DEFINE_EXCEPTION(EpipolarException)
DEFINE_EXCEPTION(ReconstructException)
DEFINE_EXCEPTION(MeasureException)
DEFINE_EXCEPTION(IOException)
```

### 单目双投影专属异常场景

| 编号 | 场景 | 触发条件 | 异常类 | 处理策略 |
|------|------|---------|--------|---------|
| E01 | 标定文件格式错误 | `.raw` 文件解析失败 | `CalibException` | 弹窗，中止 |
| E02 | 图像数量不足 | 扫描文件夹图像数 < N×3 | `ImageLoadException` | 弹窗，中止 |
| E03 | 调制度过低 | 全图 >80% 像素调制度 < 阈值 | `PhaseDecodeException` | 日志警告，继续 |
| E04 | 相位解包失败 | 三频相位矛盾无法展开 | `PhaseDecodeException` | 跳过像素，日志 |
| E05 | 极线配准失败 | 极线偏差超过 `t_max` | `EpipolarException` | 日志，尝试降级单投影 |
| E06 | 点云为空 | 有效像素数 < 100 | `ReconstructException` | 弹窗，中止保存 |
| E07 | 焊球未检测到 | Hough 检测结果为空 | `MeasureException` | bgaDataText 提示 |
| E08 | 双投影区域不重叠 | 极线匹配成功率 < 30% | `EpipolarException` | 弹窗，提示检查外参 |
| E09 | 高度超量程 | Z 超出 [depthMin, depthMax] | `ReconstructException` | 过滤超范围点，日志 |
| E10 | 保存路径不存在 | `std::filesystem::create_directories` 失败 | `IOException` | 弹窗，要求重新选择 |

### Logger 接口

```cpp
// src/common/Logger.h
class Logger {
public:
    enum Level { DEBUG, INFO, WARN, ERROR, FATAL };
    static Logger& instance();          // 单例
    void log(Level lv, const std::string& module, const std::string& msg);
    void setLogFile(const std::string& path);   // 每天一个文件
private:
    std::mutex mutex_;
    std::ofstream file_;
};
// 便捷宏
#define LOG_INFO(mod, msg)  Logger::instance().log(Logger::INFO,  mod, msg)
#define LOG_WARN(mod, msg)  Logger::instance().log(Logger::WARN,  mod, msg)
#define LOG_ERROR(mod, msg) Logger::instance().log(Logger::ERROR, mod, msg)
```

### 输出物

| 文件 | 说明 |
|------|------|
| `src/common/Exceptions.h` | 完整异常类定义 |
| `src/common/Logger.h/.cpp` | 线程安全滚动日志 |
| `docs/exception_scenarios.md` | 所有异常场景清单 + 测试用例 |

---

## Agent 6：测试与验收智能体

### 职责
1. 编写 C++ 单元测试（Google Test 或 Catch2，集成到 VS 工程 `test/` 目录）；
2. 验证工程可一键编译（Release x64）；
3. 验证所有异常场景均被正确捕获；
4. 验证 Git 推送成功；
5. 输出验收报告。

### 测试覆盖范围

| 模块 | 测试点 |
|------|--------|
| `PhaseDecoder` | 主值相位公式正确性（与 MATLAB 输出对比，误差 < 1e-6）；三频展开单调性验证 |
| `HeightCalculator` | 平面样件高度误差 < 0.01 mm |
| `PointCloudFilter` | SOR 处理后离群点数量下降验证；同一输入两次结果一致性 |
| `PointCloudIO` | PLY 写入后读回，点数、坐标误差 < 1e-5 |
| `BGADetector` | 已知图像焊球数量检测正确率 > 95% |
| `ReconstructionPipeline` | 双投影并行耗时 < 单线程运行时的 55%（验证并行有效） |
| `Logger` | 多线程并发写日志无崩溃、无行交叉 |
| `Exceptions` | 每类异常均抛出并被正确捕获，不崩溃 |
| UI | `resBtnStartRebuild` 点击后 progressBar 能从 0 增长到 100 |

### 验收标准

```markdown
# 验收报告格式
## 1. 编译验证
- [ ] Release x64 零错误、零警告（第三方库警告已抑制）
- [ ] Debug x64 可正常调试

## 2. 功能验证
- [ ] 加载标定文件成功，日志显示 "CalibLoader: Pro1 加载成功"
- [ ] 重建流水线运行到完成，点云非空（点数 > 1000）
- [ ] 点云可在 3D 视图中正常显示并旋转
- [ ] BGA 焊球检测结果显示在 bgaDataText

## 3. 性能验证
- [ ] 单次重建总耗时 < 3 s（标准测试集）
- [ ] UI 在重建过程中响应正常（可点击其他按钮）

## 4. 异常验证
- [ ] 给定损坏标定文件 → CalibException 弹窗、不崩溃
- [ ] 空文件夹 → ImageLoadException 弹窗、不崩溃

## 5. Git 验证
- [ ] `git log --oneline` 显示 "first commit"
- [ ] remote origin 指向 https://github.com/Touda8/graduateSoftware.git
```

### 输出物

| 文件 | 说明 |
|------|------|
| `test/test_phase.cpp` | PhaseDecoder 单元测试 |
| `test/test_pointcloud.cpp` | 点云处理单元测试 |
| `test/test_bga.cpp` | BGA 检测单元测试 |
| `test/test_exception.cpp` | 异常捕获测试 |
| `docs/acceptance_report.md` | 验收报告（填写实际结果） |

---

## 智能体接口约定（CONTRACT）

> 本节规定各 Agent 之间共享数据结构与交付接口，所有 Agent 必须遵守，确保并行开发后可无缝集成。

### 共享结构体（`src/common/interfaces.h` 中定义）

```cpp
// ===== 标定数据 =====
struct CalibData {
    cv::Mat allK;          // (1 x m) 相位-高度多项式系数
    cv::Mat allK_i;        // 逆映射系数
    cv::Mat X;             // 世界坐标 X 映射矩阵
    cv::Mat Y;             // 世界坐标 Y 映射矩阵
    bool isValid = false;
};

// ===== 重建参数（来自 UI）=====
struct ReconParams {
    int freq1 = 64, freq2 = 56, freq3 = 63;
    int shift1 = 8, shift2 = 8, shift3 = 8;
    double modThresh      = 0.7;
    double depthMin       = 50.0;    // mm
    double depthMax       = 210.0;   // mm
    double phaseThresh    = 0.04;
    double epiThresh      = 1.5;     // px
    double centroidThresh = 0.05;
    double tMax           = 10.0;
    double numStable      = 0.001;
    double fsnrHigh       = 0.3;
    double fsnrLow        = 0.15;
    double modHigh        = 30.0;
    double modLow         = 10.0;
    bool   orderedCloud   = false;
    int    repeatCount    = 1;
};

// ===== 重建结果 =====
struct ReconResult {
    cv::Mat absPhase1;     // Pro1 绝对相位图
    cv::Mat absPhase2;     // Pro2 绝对相位图
    cv::Mat heightMap;     // 融合高度图
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;  // 着色点云
    bool success = false;
    std::string errorMsg;
};

// ===== BGA 测量结果 =====
struct BGAMeasureResult {
    std::vector<Eigen::Vector3d> ballPositions;  // 各焊球三维坐标
    double coplanarity = 0.0;                    // 共面度 (mm)
    double maxDev      = 0.0;                    // 最大偏差
    bool   success     = false;
    std::string errorMsg;
};
```

### Agent 交付物清单与负责人

| 交付物 | 负责 Agent | 使用方 Agent |
|--------|-----------|------------|
| `docs/task_list.md` | Agent1 | Agent2/3/4/5/6 |
| `docs/ui_algorithm_binding.md` | Agent1 | Agent2, Agent3 |
| `src/common/interfaces.h` | Agent1 | Agent2/3/5 |
| `src/MainWindow.h/.cpp` | Agent2 | — |
| `src/common/Config.h/.cpp` | Agent2 | — |
| `src/visualization/VtkWidget.h/.cpp` | Agent2 | — |
| `src/reconstruction/*.h/.cpp` | Agent3 | Agent2 |
| `src/measurement/*.h/.cpp` | Agent3 | Agent2 |
| `src/pointcloud/*.h/.cpp` | Agent3 | Agent2 |
| `src/common/ThreadPool.h` | Agent3 | Agent2/3 |
| `twoProjector.sln/.vcxproj` | Agent4 | 全员 |
| `build.bat` | Agent4 | 全员 |
| `src/common/Exceptions.h` | Agent5 | Agent2/3 |
| `src/common/Logger.h/.cpp` | Agent5 | Agent2/3 |
| `test/*.cpp` | Agent6 | — |
| `docs/acceptance_report.md` | Agent6 | — |
