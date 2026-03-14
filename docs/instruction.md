# instruction.md — 项目开发总指令

## 一、开发原则（PRINCIPLE）

### 1.1 核心约束（优先级最高，所有 Agent 必须遵守）

| 编号 | 约束 | 违反后果 |
|------|------|---------|
| P01 | 严禁修改 `capWidget` 及其所有子控件（图像采集 Tab） | 导致退出验收 |
| P02 | 严禁修改 `F:\project\Envlib` 下任何文件 | 破坏共享库环境 |
| P03 | 严禁在 UI 主线程执行耗时 > 5 ms 的操作 | 导致界面卡顿 |
| P04 | 所有算法结果通过 `Qt::QueuedConnection` 信号传回 UI | 避免跨线程访问 |
| P05 | 所有公共函数必须在头文件中声明，实现在 `.cpp` | 保持接口/实现分离 |
| P06 | 使用 C++17 标准；不引入 `.lib` 之外的运行时动态依赖 | 保证构建可复现 |

### 1.2 设计原则

- **单一职责**：每个类只做一件事（`PhaseDecoder` 只解码，`Logger` 只写日志）。
- **依赖倒置**：高层模块（UI/Pipeline）依赖抽象接口（`IDecoder`, `IFilter`），不依赖具体实现。
- **最小可见**：所有成员变量为 `private`；辅助函数为 `private`；只暴露必要的公共接口。
- **异常安全**：析构函数不抛异常；资源均通过 RAII 管理（`cv::Mat`, `pcl::PointCloud::Ptr`, Qt 对象树）。
- **不过度设计**：不为"可能的未来功能"预留接口；当前任务范围之外的扩展不实现。

---

## 二、构建规范（BUILD）

### 2.1 工程结构

```
twoProjector.sln
twoProjector.vcxproj
twoProjector.vcxproj.props     ← 库路径属性表（任何人拉取后只改这个）
twoProjector.vcxproj.filters   ← 源文件分组（src按目录分组）
```

### 2.2 工程属性（Release|x64 基准）

| 属性 | 值 |
|------|----|
| 目标平台 | x64 |
| 工具集 | v142 |
| C++ 标准 | `/std:c++17` |
| 运行时库 | `/MD`（Release） / `/MDd`（Debug） |
| 字符集 | Unicode (`/D UNICODE /D _UNICODE`) |
| 附加宏 | `NOMINMAX;_USE_MATH_DEFINES;_CRT_SECURE_NO_WARNINGS` |
| 多进程编译 | `/MP` |
| 警告级别 | `/W3`；第三方库目录使用 `/external:I` 抑制警告 |
| 优化 | Release: `/O2`；Debug: `/Od /Zi` |
| Qt 集成 | Qt VS Tools，项目类型 Qt Application |

### 2.3 库引用顺序（Link → AdditionalDependencies）

```
# OpenCV（Release 无 d 后缀）
opencv_world455.lib

# PCL 核心
pcl_common_release.lib; pcl_io_release.lib; pcl_filters_release.lib;
pcl_features_release.lib; pcl_segmentation_release.lib;
pcl_registration_release.lib; pcl_visualization_release.lib

# PCL 3rdParty（由 PCL CMake 链接过，通常不需要手动加，但若缺失则补）
vtkRenderingCore-9.1.lib; vtkInteractionStyle-9.1.lib;
vtkRenderingOpenGL2-9.1.lib; vtkGUISupportQt-9.1.lib;
flann_cpp_s.lib; qhullstatic_r.lib

# Boost（仅用到 Boost.Thread, Boost.Filesystem）
libboost_thread-vc142-mt-x64-1_78.lib;
libboost_filesystem-vc142-mt-x64-1_78.lib;
libboost_system-vc142-mt-x64-1_78.lib

# Ceres
ceres.lib; glog.lib
```

> **注意**：Debug 配置下 OpenCV 库名为 `opencv_world455d.lib`，PCL 库名包含 `debug` 字样。  
> 统一做法：用 `$(Configuration)` 条件区分 Release/Debug 链接。

### 2.4 运行时 DLL 部署

首次运行前将以下目录加入系统 PATH，或复制 DLL 到 exe 目录：

```
F:\project\Envlib\opencv455\opencv\x64\vc16\bin
F:\project\Envlib\PCL1.12.1\bin
F:\project\Envlib\PCL1.12.1\3rdParty\VTK\bin
F:\project\Envlib\PCL1.12.1\3rdParty\Boost\lib
F:\project\Envlib\PCL1.12.1\3rdParty\OpenNI2\Redist
```

### 2.5 快速构建

```bat
:: build.bat
@echo off
set VS=C:\Program Files (x86)\Microsoft Visual Studio\2019
call "%VS%\Common7\Tools\VsDevCmd.bat" -arch=x64
msbuild twoProjector.sln /p:Configuration=Release /p:Platform=x64 /m /nologo
if %errorlevel%==0 ( echo [OK] Build succeeded ) else ( echo [FAIL] Build failed )
pause
```

---

## 三、UI 开发规范（UI_RULE）

### 3.1 绝对禁区

以下内容**不论任何情况均不得修改**：
- `twoProjector.ui` 中 `capWidget` 及其所有直接/间接子控件；
- `ui_twoProjector.h`（uic 自动生成，每次构建覆盖，不要手动编辑）；
- `capWidget` 相关的信号槽连接（即使 `connect` 语句写在 `MainWindow.cpp` 中）。

### 3.2 控件命名约定（已在 .ui 中固定，不可更改）

| 前缀 | 所属 Tab | 示例 |
|------|---------|------|
| `cap*` | 图像采集（禁区） | `capBtnSoftTrigger` |
| `res*` | 系统重建 | `resBtnStartRebuild`, `resSpinFreq1` |
| `pc*` | 点云处理（resWidget 子区域） | `pcBtnSOR`, `pcSpinVoxelSize` |
| `bga*` | BGA 共面度测量 | `bgaBtnStart`, `bgaSpinCount` |

### 3.3 信号槽规范

```cpp
// [必须] 跨线程更新 UI 使用 Qt::QueuedConnection
connect(worker, &Worker::progressUpdated,
        ui->resProgressBar, &QProgressBar::setValue,
        Qt::QueuedConnection);

// [禁止] 在工作线程直接操作控件
// worker 中绝对不允许出现：
ui->resProgressBar->setValue(50);   // ← 禁止！
resLogText->append("...");           // ← 禁止！
```

### 3.4 参数持久化

- 所有 UI 参数自动保存到 `config/params.json`（应用启动时加载，关闭时保存）。
- 格式：以控件 `objectName` 为 key，控件当前值为 value。
- 使用 `Config::save(ui)` / `Config::load(ui)` 统一处理（Agent2 实现）。

```json
{
  "resSpinFreq1": 64,
  "resSpinFreq2": 56,
  "resSpinFreq3": 63,
  "resDblSpinModThresh": 0.7,
  "resEditDepthMin": "50",
  "resEditDepthMax": "210",
  "resDblSpinEpiThresh": 1.5
}
```

### 3.5 VTK 嵌入规范

- `resCloudLabel` 在 `.ui` 中为占位 `QLabel`，运行时替换为 `QVTKOpenGLNativeWidget`；
- 使用 `vtkRenderer` + `vtkRenderWindowInteractor` 实现交互；
- 点云颜色渲染：高度值映射为 Jet 色表；
- 视图操作（frontView / topView / isoView 等 8 个按钮）通过调用 `vtkCamera` API 实现。

### 3.6 进度条与日志

```cpp
// 进度条（resProgressBar）：重建开始时 setValue(0)，完成时 setValue(100）
// 日志（resLogText）：追加带时间戳的消息
void MainWindow::appendLog(const QString& msg) {
    const auto stamp = QDateTime::currentDateTime().toString("[hh:mm:ss] ");
    ui->resLogText->append(stamp + msg);
}
```

---

## 四、算法翻译规范（ALGO_RULE）

### 4.1 MATLAB → C++ 翻译对照

| MATLAB 文件 | C++ 目标文件 | 关键 API 变化 |
|------------|------------|--------------|
| `CalculatePhaseMatrix.m` | `PhaseDecoder::calcPhaseMatrix()` | `cv::Mat` 替代矩阵；`atan2f` 替代 `atan2` |
| `Multifrequency_phase.m` | `PhaseDecoder::multiFreqUnwrap()` | 整数截断用 `static_cast<int>(std::floor(...))` |
| `CalcFringeModulation.m` | `PhaseDecoder::calcFringeModulation()` | `cv::sqrt`, `cv::pow` 替代 element-wise 操作 |
| `CalcModulationContrast.m` | `PhaseDecoder::calcModulationContrast()` | 同上 |
| `CalcPhaseErrorEnergy.m` | `PhaseDecoder::calcPhaseErrorEnergy()` | `cv::norm` 替代 `norm()` |
| `UNwrapPhase.m` | `PhaseDecoder::unwrapPhase()` | 空间相位展开，使用像素队列迭代 |
| `CalculatePlaneHeight.m` | `HeightCalculator::calcPlaneHeight()` | 多项式求值用 `cv::Mat` 矩阵乘法 |
| `DepthGradientFilter.m` | `HeightCalculator::depthGradientFilter()` | `cv::Sobel` 替代 `gradient()` |
| `PhaseQualityMask.m` | `HeightCalculator::phaseQualityMask()` | `cv::threshold` |
| `Generation_of_mask.m` | `HeightCalculator::generateMask()` | `cv::inRange` |
| `fitPlane_PCA.m` | `GeometryAnalyzer::fitPlanePCA()` | `pcl::PCA<>` |
| `PCALeastSquaresFittingPlane.m` | `GeometryAnalyzer::fitPlaneLS()` | `Eigen::JacobiSVD` |
| `BGA_2D_BallDetection.m` | `BGADetector::detect2DBalls()` | `cv::HoughCircles` |
| `BGA_3D_VertexLocalization.m` | `BGADetector::localize3DBalls()` | PCL KD-tree 检索 |
| `reconstruction_SNR.m` | `ReconstructionPipeline::calcSNR()` | 评估函数，Debug 模式输出 |

### 4.2 数据类型映射

| MATLAB | C++ |
|--------|-----|
| `double` matrix | `cv::Mat` (CV_64F) |
| `uint8` 图像 | `cv::Mat` (CV_8U) |
| cell array of mats | `std::vector<cv::Mat>` |
| struct | C++ struct |
| `NaN` | `std::numeric_limits<float>::quiet_NaN()` |
| `Inf` | `std::numeric_limits<float>::infinity()` |
| `linspace(a,b,n)` | `std::ranges::iota_view` 或手动计算 |
| logical mask | `cv::Mat` (CV_8U，非零为 true) |

### 4.3 翻译正确性验证

每个翻译函数必须通过以下验证：
1. 用相同输入（固定随机种子）分别运行 MATLAB 和 C++；
2. 所有输出像素的绝对差 < `1e-5`（`double` 精度）；
3. 验证脚本放在 `test/verify_translation.m`（MATLAB 端）和 `test/test_translation.cpp`（C++ 端）。

---

## 五、线程规范（THREAD_RULE）

### 5.1 线程架构

```
Qt 主线程（UI 线程）
    │
    ├─ MainWindow（UI 事件、信号槽分发）
    │
    └─ 发射 startRecon / startMeasure 信号
           │
           ▼
    ThreadPool（std::thread，核数 - 1）
           │
           ├─ Task A：读取 Pro1 图像（I/O 密集）
           ├─ Task B：读取 Pro2 图像（与 A 并行）
           ├─ Task C：Pro1 相位解码（CPU 密集，A 完成后）
           ├─ Task D：Pro2 相位解码（CPU 密集，B 完成后）
           ├─ Task E：极线配准（C+D 完成后）
           ├─ Task F：高度计算 + 点云生成（E 完成后）
           └─ Task G：点云后处理子任务（F 完成后，可再并行）
```

### 5.2 ThreadPool 接口

```cpp
// src/common/ThreadPool.h（header-only）
class ThreadPool {
public:
    explicit ThreadPool(size_t nThreads = std::thread::hardware_concurrency() - 1);
    ~ThreadPool();                              // 等待所有任务完成

    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args)          // 返回 std::future<返回类型>
        -> std::future<std::invoke_result_t<F, Args...>>;

    void waitAll();                             // 阻塞直到队列清空
};
```

### 5.3 线程安全规则

| 规则 | 说明 |
|------|------|
| T01 | **只有 UI 线程可以调用任何 `QWidget::*` 方法** |
| T02 | 工作线程结果通过 `emit signal(...)` 传回 UI，不能直接操作控件 |
| T03 | `cv::Mat` 共享只读数据时无需加锁；如需写，各任务使用独立副本 |
| T04 | `pcl::PointCloud::Ptr` 采用 `boost::shared_ptr`，跨线程传递前完成写操作 |
| T05 | 日志写入 (`Logger`) 内部加 `std::mutex`，调用方无需额外加锁 |
| T06 | 重建期间禁用 `resBtnStartRebuild`（在 `onStartRebuild` 开始时 `setEnabled(false)`，`finished` 信号后恢复）|

### 5.4 并行任务同步示例

```cpp
// ReconstructionPipeline::runAsync 内部实现参考
void ReconstructionPipeline::runAsync(const QString& folder) {
    std::thread([this, folder]() {
        try {
            auto futA = pool_.submit([&]{ return loadImages(folder, 0); });
            auto futB = pool_.submit([&]{ return loadImages(folder, 1); });
            auto imgsA = futA.get();   // 等 Pro1 图像
            auto imgsB = futB.get();   // 等 Pro2 图像
            emit progressUpdated(20);

            auto futC = pool_.submit([&]{ return decodePhase(imgsA); });
            auto futD = pool_.submit([&]{ return decodePhase(imgsB); });
            auto phi1 = futC.get();
            auto phi2 = futD.get();
            emit progressUpdated(60);

            auto cloud = calcCloud(phi1, phi2);
            emit progressUpdated(100);
            emit cloudReady(cloud);
            emit finished(true, "");
        } catch (const AppException& e) {
            emit finished(false, QString::fromStdString(e.what()));
        }
    }).detach();
}
```

---

## 六、文件规范（FILE_RULE）

### 6.1 目录结构（强制执行）

```
graduateSoftware/
├── src/                          # C++ 源文件（禁止平铺，按模块分子目录）
│   ├── MainWindow.h/.cpp
│   ├── common/
│   │   ├── interfaces.h          # 抽象接口（纯虚类/结构体）
│   │   ├── Exceptions.h          # 异常类层次
│   │   ├── Logger.h/.cpp         # 线程安全日志
│   │   ├── ThreadPool.h          # 线程池（header-only）
│   │   └── Config.h/.cpp         # JSON 参数读写
│   ├── reconstruction/
│   │   ├── PhaseDecoder.h/.cpp
│   │   ├── HeightCalculator.h/.cpp
│   │   ├── ReconstructionPipeline.h/.cpp
│   │   └── CalibLoader.h/.cpp
│   ├── measurement/
│   │   ├── BGADetector.h/.cpp
│   │   └── BGAMeasurePipeline.h/.cpp
│   ├── pointcloud/
│   │   ├── PointCloudFilter.h/.cpp
│   │   ├── PointCloudIO.h/.cpp
│   │   └── GeometryAnalyzer.h/.cpp
│   └── visualization/
│       └── VtkWidget.h/.cpp
├── algorithm/                    # 原始 MATLAB 文件（只读参考）
├── data/                         # 标定数据、测量结果（不纳入 Git）
├── config/
│   └── params.json               # UI 参数持久化
├── logs/                         # 运行日志（每天一个文件，不纳入 Git）
├── test/                         # 单元测试
├── docs/                         # 本套文档
├── res/                          # 资源文件（图标、样式表）
├── twoProjector.ui               # Qt Designer 界面文件（禁止手动编辑后直接运行重新 uic）
├── twoProjector.sln
├── twoProjector.vcxproj
├── twoProjector.vcxproj.props
├── build.bat
└── .gitignore
```

### 6.2 文件规范细则

| 规则 | 说明 |
|------|------|
| F01 | 每个 `.cpp` 首行 `#include` 对应的 `.h` | 单元编译完整性 |
| F02 | 所有头文件使用 `#pragma once` | 防止重复包含 |
| F03 | 第三方库头文件使用 `<angle brackets>`，本项目头文件使用 `"quotes"` | 区分来源 |
| F04 | 不在头文件中 `using namespace std;` | 防止污染包含方 |
| F05 | 日志文件命名：`logs/YYYYMMDD.log`，超过 10 MB 滚动为 `.1.log` | |
| F06 | 配置文件 `config/params.json` 必须保持合法 JSON，损坏时重置为默认值 | |

---

## 七、异常处理规范（EXCEPTION_RULE）

### 7.1 异常层次（详见 `src/common/Exceptions.h`）

```
AppException
├── CalibException         # 标定文件加载/解析失败
├── ImageLoadException     # 图像文件读取失败或数量不足
├── PhaseDecodeException   # 相位解码失败（调制度过低、相位矛盾）
├── EpipolarException      # 极线配准失败（偏差超阈值、重叠率过低）
├── ReconstructException   # 点云生成失败（空云、超量程）
├── MeasureException       # BGA 检测失败（焊球未检测到）
└── IOException            # 文件保存/创建目录失败
```

### 7.2 强制捕获点

所有信号槽函数（`MainWindow` 中的 `on*` 槽）必须包裹 try/catch：

```cpp
void MainWindow::onStartRebuild() {
    try {
        ui->resBtnStartRebuild->setEnabled(false);
        // 启动工作线程...
    } catch (const AppException& e) {
        QMessageBox::critical(this,
            tr("重建失败"),
            tr("%1：%2").arg(
                QString::fromStdString(e.module()),
                QString::fromStdString(e.what())));
        LOG_ERROR("MainWindow", e.what());
        ui->resBtnStartRebuild->setEnabled(true);
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("未知错误"), e.what());
        LOG_ERROR("MainWindow", std::string("std::exception: ") + e.what());
    }
}
```

### 7.3 日志级别使用规范

| 事件严重度 | 使用级别 | 是否弹窗 |
|-----------|---------|---------|
| 只是信息（加载成功、步骤完成） | `LOG_INFO` | 否，仅写日志和 resLogText |
| 可恢复的异常（部分像素跳过） | `LOG_WARN` | 否，日志 + resLogText 变色提示 |
| 功能中断（算法失败） | `LOG_ERROR` | 是，`QMessageBox::critical` |
| 程序无法继续 | `LOG_FATAL` | 是，弹窗后退出 |

### 7.4 析构安全

- 所有 `std::thread`/`std::future` 相关资源在对象析构前 `join()`/`wait()`；
- `cv::Mat` 引用计数自动管理，不需要手动 `release()`；
- PCL 点云使用 `pcl::PointCloud<T>::Ptr`（`boost::shared_ptr`）。

---

## 八、命名规范（NAMING）

### 8.1 通用规则

| 类别 | 规范 | 示例 |
|------|------|------|
| 类名 | UpperCamelCase | `PhaseDecoder`, `BGADetector` |
| 函数名 | lowerCamelCase | `calcPhaseMatrix()`, `runAsync()` |
| 成员变量 | `lowerCamel_` 或 `lowerCamel`（统一用尾下划线私有成员） | `modThresh_`, `pool_` |
| 局部变量 | lowerCamelCase | `phaseMap`, `cloudPtr` |
| 常量 / 枚举 | `k` 前缀 + UpperCamelCase 或 `ALL_CAPS` | `kMaxFreq`, `DEFAULT_MOD_THRESH` |
| 文件名 | UpperCamelCase（与主类名对应） | `PhaseDecoder.h`, `BGADetector.cpp` |
| 命名空间 | 小写，单词 | `recon`, `meas`, `pcproc` |

### 8.2 Qt 特定约定

```cpp
// 信号：动词 + Past/Ready 形式
signals:
    void progressUpdated(int percent);
    void cloudReady(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void logMessage(const QString& msg);

// 槽：on + 控件名 + 动作（由 Qt 自动连接时）
private slots:
    void on_resBtnStartRebuild_clicked();   // 自动连接
    void onStartRebuild();                 // 手动连接也可

// Qt 属性命名：遵照 .ui 中已有名称，禁止更改
```

### 8.3 算法参数命名（与 MATLAB 对应）

| MATLAB 变量 | C++ 成员/参数 | 含义 |
|------------|-------------|------|
| `f1, f2, f3` | `freq1, freq2, freq3` | 三个条纹频率 |
| `N` | `nShift` | 相移步数 |
| `ModThresh` | `modThresh` | 调制度阈值 |
| `allK` | `allK` (Eigen::MatrixXd) | 相位-高度多项式系数 |
| `I_masked` | `maskedImage` | 掩膜后图像 |
| `phi` | `phase` | 主值相位（以 `phase` 命名避免 `phi` 歧义） |
| `ABS_phi` | `absPhase` | 绝对展开相位 |

---

## 九、Git 规范（GIT_RULE）

### 9.1 初始化

```bash
git init
git remote add origin https://github.com/Touda8/graduateSoftware.git
git add .
git commit -m "chore: initial project scaffold"
git push -u origin main
```

### 9.2 .gitignore（完整版）

```gitignore
# Visual Studio
.vs/
*.suo
*.user
*.userosscache
*.sln.docstates
x64/
Debug/
Release/
ipch/
*.aps

# build 产物
build/
*.exe
*.dll
*.pdb
*.ilk
*.obj
*.exp
*.lib

# Qt uic/moc 生成
ui_*.h
moc_*.cpp
qrc_*.cpp
*.qm

# 数据与日志（体积大 / 敏感）
data/
logs/
*.mat
*.pcd
*.ply

# 配置（含本地路径，不共享）
config/params.json

# Python / OS
__pycache__/
*.pyc
Thumbs.db
.DS_Store
```

### 9.3 提交信息格式

```
<type>(<scope>): <简短描述>

[可选：详细说明]
```

| type | 含义 |
|------|------|
| `feat` | 新功能 |
| `fix` | Bug 修复 |
| `refactor` | 重构（不改功能） |
| `docs` | 文档更新 |
| `test` | 测试用例 |
| `chore` | 构建/配置 |
| `perf` | 性能优化 |

**示例**：
```
feat(reconstruction): implement multi-frequency phase unwrapping

Translated from algorithm/unwrap/Multifrequency_phase.m.
Passes accuracy test (max error < 1e-5) against MATLAB reference output.
```

### 9.4 分支策略

| 分支 | 用途 |
|------|------|
| `main` | 可发布版本，只接受 PR 合并 |
| `dev` | 日常开发主分支 |
| `feat/reconstruction` | Agent3 算法开发 |
| `feat/ui` | Agent2 UI 开发 |
| `feat/build` | Agent4 工程化 |
| `fix/*` | Bug 修复分支 |

> 当前开发阶段（单人研究项目）可简化为直接推 `main`，但提交信息格式必须遵守。
