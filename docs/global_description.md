# 全局项目描述：单目双投影仪三维重建与点云测量系统

## 1. 项目基本信息

| 项目 | 内容 |
|------|------|
| 项目名称 | graduateSoftware |
| 窗口标题 | 远心单目双光栅三维重建测量系统 |
| 业务场景 | 单目相机 + 双投影仪结构光三维重建与点云测量 |
| 开发语言 | C++17 |
| UI 框架 | Qt（与 VS2019 原生集成），基于已有 `twoProjector.ui` (Qt 4.0 格式) |
| 主窗口类名 | `twoProjectorClass`（继承 `QMainWindow`） |
| 分辨率 | 主窗口固定 1644×1000 像素 |
| 开发环境 | Visual Studio 2019，平台工具集 v142，目标平台 x64 |
| C++ 标准 | C++17（`/std:c++17`） |
| 远程仓库 | https://github.com/Touda8/graduateSoftware.git（main 分支） |

### Tab 结构（严格来源于 twoProjector.ui）

| Tab 索引 | 对象名 | 标题 | 开发权限 |
|----------|--------|------|----------|
| 0 | `capWidget` | 图像采集 | **禁止修改** |
| 1 | `resWidget` | 系统重建 | **重点开发** |
| 2 | `bgaWidget` | BGA共面度测量 | **重点开发** |

---

## 2. 开发环境与依赖库

### 2.1 编译环境

| 项目 | 值 |
|------|-----|
| IDE | Visual Studio 2019 |
| 安装路径 | `C:\Program Files (x86)\Microsoft Visual Studio\2019` |
| 工具集 | v142 |
| 目标平台 | x64 |
| C++ 标准 | `/std:c++17` |
| 运行时库 | `/MD`（Release），`/MDd`（Debug） |

### 2.2 已验证可用库（全部位于 `F:\project\Envlib`）

| 库 | 目录名 | 用途 | Include 路径 | Lib 路径 |
|----|--------|------|-------------|---------|
| OpenCV | `opencv455` | 图像处理、相位计算 | `opencv455\opencv\include` | `opencv455\opencv\x64\vc16\lib` |
| PCL | `PCL1.12.1` | 点云处理、滤波、测量 | `PCL1.12.1\include\pcl-1.12` | `PCL1.12.1\lib` |
| Eigen | `eigen-3.4.0` | 线性代数、矩阵运算 | `eigen-3.4.0` | —（纯头文件） |
| Boost | `PCL1.12.1\3rdParty\Boost` | PCL 依赖 | `...\Boost\include\boost-1_78` | `...\Boost\lib` |
| FLANN | `PCL1.12.1\3rdParty\FLANN` | PCL 近邻搜索 | `...\FLANN\include` | `...\FLANN\lib` |
| VTK | `PCL1.12.1\3rdParty\VTK` | 点云可视化 | `...\VTK\include\vtk-9.1` | `...\VTK\lib` |
| Qhull | `PCL1.12.1\3rdParty\Qhull` | 凸包计算 | `...\Qhull\include` | `...\Qhull\lib` |
| Ceres | `ceres` | 非线性优化（如标定） | `ceres\include` | `ceres\lib` |

> **约定**：所有库只能调用，禁止修改或删除任何库文件。

### 2.3 Qt 配置

- Qt 与 VS2019 通过 **Qt VS Tools** 插件集成；
- `twoProjector.ui` 通过 **uic** 自动生成 `ui_twoProjector.h`，禁止手动修改；
- 信号槽统一在对应 `.cpp` 中用 `connect()` 声明，不使用 SIGNAL/SLOT 宏。

---

## 3. UI 控件 → 功能绑定清单

### 3.1 系统重建 Tab（`resWidget`）

#### 输入控件（参数读取）

| 控件名 | 类型 | 默认值 | 含义 |
|--------|------|--------|------|
| `resEditCalPro1` | QLineEdit | `D:/Calibration/pro1_calib.raw` | 投影仪1标定文件路径 |
| `resEditCalPro2` | QLineEdit | `D:/Calibration/pro2_calib.raw` | 投影仪2标定文件路径 |
| `resEditScanFolder` | QLineEdit | `D:/Scan/.../bga` | 扫描图像文件夹路径 |
| `resEditEpiFile` | QLineEdit | `D:/Calibration/epipolar_params.mat` | 极线参数文件路径 |
| `resComboDecodeType` | QComboBox | `multi_freq` | 解码算法类型 |
| `resCheckOrdered` | QCheckBox | true | 是否生成有序点云 |
| `resSpinFreq1/2/3` | QSpinBox | 64 / 56 / 63 | 三个频率周期数 |
| `resSpinShift1/2/3` | QSpinBox | 8 / 8 / 8 | 三个频率相移步数 N |
| `resDblSpinModThresh` | QDoubleSpinBox | 0.7 | 调制度阈值 |
| `resCheckDepthRange` | QCheckBox | true | 是否启用深度范围限制 |
| `resEditDepthMin/Max` | QLineEdit | 50.0 / 210.0 mm | 有效深度范围 |
| `resEditPhaseThresh` | QLineEdit | 0.040 | 相位差阈值 |
| `resDblSpinEpiThresh` | QDoubleSpinBox | 1.5 px | 极线偏差阈值 τ_epi |
| `resDblSpinCentroidThresh` | QDoubleSpinBox | 0.05 | 质心偏移阈值 τ_μ |
| `resDblSpinTmax` | QDoubleSpinBox | 10.0 | 最大修正幅度 t_max |
| `resDblSpinNumStable` | QDoubleSpinBox | 0.001 | 数值稳定阈值 ε |
| `resDblSpinFSNRHigh/Low` | QDoubleSpinBox | 0.30 / 0.15 | FSNR 区域判定阈值 |
| `resDblSpinModHigh/Low` | QDoubleSpinBox | 30 / 10 | 调制度区域判定阈值 |
| `resComboSaveMode` | QComboBox | 单次重建 | 重建模式 |
| `resSpinRepeatCount` | QSpinBox | 1 | 多次重建次数 |
| `resEditSavePath` | QLineEdit | `D:/Scan/.../info` | 点云保存路径 |

#### 操作按钮 → 功能绑定

| 按钮控件名 | 文本 | 触发功能 | 调用算法/模块 |
|------------|------|----------|--------------|
| `resBtnCalPro1Browse` | 浏览 | 打开文件对话框，更新 `resEditCalPro1` | QFileDialog |
| `resBtnCalPro2Browse` | 浏览 | 打开文件对话框，更新 `resEditCalPro2` | QFileDialog |
| `resBtnScanBrowse` | 浏览 | 打开目录对话框，更新 `resEditScanFolder` | QFileDialog |
| `resBtnEpiBrowse` | 浏览 | 打开文件对话框，更新 `resEditEpiFile` | QFileDialog |
| `resBtnSavePathBrowse` | 浏览 | 打开目录对话框，更新 `resEditSavePath` | QFileDialog |
| `resBtnLoadCalib` | 加载标定文件 | 读取 Pro1/Pro2 `.raw` 标定文件、极线参数，解析为内参/外参矩阵 | `CalibLoader` |
| `resBtnStartRebuild` | 开始重建 | 启动完整重建流水线（异步，线程池） | `ReconstructionPipeline` |
| `resBtnSaveCloud` | 保存点云 | 将当前点云写入 PLY/PCD/CSV，路径来自 `resEditSavePath` | `PointCloudIO` |
| `resBtnTogglePanel` | 收起/展开参数面板 | 切换 `resLeftWidget` 的可见性 | UI 逻辑 |

#### 3D 点云处理操作按钮

| 按钮控件名 | 触发功能 | 调用模块 |
|------------|----------|---------|
| `pcBtnImportCloud` | 导入点云文件 | `PointCloudIO::load()` |
| `pcBtnCropROI` | ROI 框选裁剪 | `pcl::CropBox` |
| `pcBtnCropPlane` | 沿平面裁剪 | `pcl::PlaneClipper3D` |
| `pcBtnSubsample` | 体素降采样 | `pcl::VoxelGrid`，体素尺寸来自 `pcSpinVoxelSize` |
| `pcBtnMerge` | 合并 Pro1+Pro2 点云 | `PointCloudMerger` |
| `pcBtnDuplicate` | 去除重复点 | `pcl::UniformSampling` |
| `pcBtnSOR` | SOR 统计滤波 | `pcl::StatisticalOutlierRemoval`，K=`pcSpinSORK`，σ=`pcSpinSORStd` |
| `pcBtnROR` | ROR 半径滤波 | `pcl::RadiusOutlierRemoval`，R=`pcSpinRORRadius`，N=`pcSpinRORMin` |
| `pcBtnPassThrough` | 直通滤波 | `pcl::PassThrough`，轴/范围来自对应控件 |
| `pcBtnGaussian` | 高斯平滑 | `pcl::MovingLeastSquares` |
| `pcBtnFitPlane` | 平面拟合 | PCA/RANSAC/最小二乘，方法来自 `pcComboFitMethod` |
| `pcBtnNormal` | 法线估计 | `pcl::NormalEstimationOMP`，半径来自 `pcSpinNormalR` |
| `pcBtnCurvature` | 曲率计算 | `pcl::PrincipalCurvaturesEstimation` |
| `pcBtnRoughness` | 粗糙度估计 | 局部平面残差 |
| `pcBtnICP` | ICP 配准 | `pcl::IterativeClosestPoint` |
| `pcBtnDistance` | 距离映射 | 点到参考平面有符号距离 |
| `pcBtnPCAAlign` | PCA 对齐 | `pcl::PCA` |
| `pcBtnCenter` | 居中原点 | 平移质心至原点 |
| `pcBtnLevelPlane` | 基准平面校正 | 拟合平面旋转至水平 |
| `pcBtnApplyTransform` | 应用旋转平移 | `pcl::transformPointCloud`，参数来自 `pcSpinRot/Trans XYZ` |
| `pcBtnViewFront/Back/…` | 视图切换 | VTK camera preset |
| `pcBtnExportPLY/PCD/CSV` | 导出点云 | `PointCloudIO::save()` |
| `pcBtnScreenshot` | 视图截图 | VTK `vtkWindowToImageFilter` |

#### 显示控件

| 控件名 | 用途 |
|--------|------|
| `resPhaseLabel1` | 显示 Pro1 绝对相位图（QLabel + QPixmap） |
| `resPhaseLabel2` | 显示 Pro2 绝对相位图 |
| `resCloudLabel` | 3D 点云渲染区（嵌入 VTK QVTKOpenGLNativeWidget） |
| `resProgressBar` | 重建进度 0~100% |
| `resLogText` | 重建日志文本（追加模式，线程安全） |
| `pcCloudListWidget` | 已导入点云列表（右键菜单：重命名/删除/显隐） |

---

### 3.2 BGA 共面度测量 Tab（`bgaWidget`）

| 控件名 | 类型 | 含义 |
|--------|------|------|
| `bgaSpinCount` | QSpinBox | 测量次数（1~100） |
| `bgaBtnStart` | QPushButton | 开始测量 → 调用 `BGAMeasurePipeline` |
| `bgaBtnPlot` | QPushButton | 数据画图 → 调用 `BGAPlotter` |
| `bgaBtnBallShow` | QPushButton | 焊球定位显示 → 叠加在点云视图上 |
| `bgaBtnImport` | QPushButton | 数据导入 → 读取已有测量 `.csv` |
| `bgaBtnSave` | QPushButton | 结果保存 → 导出 `.csv`/`.txt` |
| `bgaDataText` | QTextEdit | 测量结果文本展示 |

---

## 4. 算法规则

### 4.1 MATLAB → C++ 翻译对照（来源于 `algorithm/` 目录）

| MATLAB 文件 | C++ 目标类/函数 | 所属模块 |
|-------------|----------------|---------|
| `unwrap/Multifrequency_phase.m` | `PhaseDecoder::multiFreqUnwrap()` | 相位解包裹 |
| `unwrap/CalculatePhaseMatrix.m` | `PhaseDecoder::calcPhaseMatrix()` | 相位矩阵计算 |
| `unwrap/CalcFringeModulation.m` | `PhaseDecoder::calcFringeModulation()` | 条纹调制度 |
| `unwrap/CalcModulationContrast.m` | `PhaseDecoder::calcModulationContrast()` | 调制度对比度 |
| `unwrap/CalcPhaseErrorEnergy.m` | `PhaseDecoder::calcPhaseErrorEnergy()` | 相位误差能量 |
| `unwrap/UNwrapPhase.m` | `PhaseDecoder::unwrapPhase()` | 相位展开 |
| `unwrap/CalculatePlaneHeight.m` | `HeightCalculator::calcPlaneHeight()` | 高度计算 |
| `unwrap/DepthGradientFilter.m` | `PointCloudFilter::depthGradientFilter()` | 深度梯度滤波 |
| `unwrap/PhaseQualityMask.m` | `PointCloudFilter::phaseQualityMask()` | 相位质量掩膜 |
| `unwrap/Generation_of_mask.m` | `PointCloudFilter::generateMask()` | 掩膜生成 |
| `unwrap/fitPlane_PCA.m` | `GeometryAnalyzer::fitPlanePCA()` | PCA 平面拟合 |
| `unwrap/PCALeastSquaresFittingPlane.m` | `GeometryAnalyzer::fitPlaneLeastSquares()` | 最小二乘平面 |
| `Chip/BGA/segment/BGA_2D_BallDetection.m` | `BGADetector::detect2DBalls()` | BGA 二维检测 |
| `Chip/BGA/pointCal/BGA_3D_VertexLocalization.m` | `BGADetector::localize3DBalls()` | BGA 三维定位 |
| `Chip/four/segment/QFP_Seg_Step1~5_*.m` | `QFPSegmentor::step1~5()` | QFP 分割流水 |
| `Chip/four/pointCal/QFP_Cal_Step1~4_*.m` | `QFPCalculator::step1~4()` | QFP 共面度计算 |
| `restruction/reconstruction_SNR.m` | `ReconstructionPipeline::run()` | 重建主入口 |

### 4.2 输入约定

- 所有图像以 `cv::Mat` 传递（double 精度，`CV_64F`）；
- 三频八步图像以 `std::array<std::array<cv::Mat, 8>, 3>` 组织（外层频率，内层步数）；
- 标定数据以自定义 `CalibData` 结构体传递（详见 Agent 3 接口文档）；
- 高度输出以 `cv::Mat`（`CV_64F`）或 `pcl::PointCloud<pcl::PointXYZ>` 双接口。

### 4.3 性能目标

| 环节 | 目标时间 |
|------|---------|
| 单投影仪相位解码（1套3×8图） | < 500 ms |
| 双投影仪并行解码 | < 600 ms（并行） |
| 点云重建 | < 1 s |
| 点云滤波（SOR/ROR） | < 200 ms |
| BGA 焊球定位 | < 500 ms |
| UI 响应延迟 | < 200 ms |

---

## 5. 多线程架构

```
UI 线程（Qt main thread）
    │
    ├─ QThreadPool / std::thread pool（工作线程池，4~8 线程）
    │      ├─ Task: 图像读取（I/O 密集，可并行）
    │      ├─ Task: Pro1 相位解码（与 Pro2 并行）
    │      ├─ Task: Pro2 相位解码（与 Pro1 并行）
    │      ├─ Task: 极线修正（串行，依赖双相位）
    │      ├─ Task: 高度计算（串行，依赖极线修正）
    │      ├─ Task: 点云生成（串行）
    │      └─ Task: 点云后处理（SOR/ROR 等，可并行）
    │
    └─ 信号槽回调 → 更新 UI（progressBar、logText、phaseLabel）
```

- 线程池实现：`std::thread` + 无锁任务队列，或直接使用 `QThreadPool` + `QRunnable`；
- 跨线程通信：`QMetaObject::invokeMethod(..., Qt::QueuedConnection)` 或 `emit signal` 回主线程；
- 禁止：在工作线程中直接访问任何 Qt 控件；
- 进度上报：每完成一个 Task，通过 `emit progressUpdated(int percent)` 通知主线程更新 `resProgressBar`。

---

## 6. 项目目录结构（强制约定）

```
F:\project\graduateSoftware\
├── twoProjector.ui              # 原始 UI 文件，禁止直接编辑（通过 uic 生成头文件）
├── src/                         # C++ 源码
│   ├── main.cpp
│   ├── MainWindow.h / .cpp      # 主窗口，信号槽绑定
│   ├── reconstruction/          # 重建模块
│   │   ├── ReconstructionPipeline.h / .cpp
│   │   ├── PhaseDecoder.h / .cpp
│   │   ├── HeightCalculator.h / .cpp
│   │   └── CalibLoader.h / .cpp
│   ├── pointcloud/              # 点云处理模块
│   │   ├── PointCloudIO.h / .cpp
│   │   ├── PointCloudFilter.h / .cpp
│   │   ├── PointCloudMerger.h / .cpp
│   │   └── GeometryAnalyzer.h / .cpp
│   ├── measurement/             # 测量模块
│   │   ├── BGADetector.h / .cpp
│   │   └── BGAMeasurePipeline.h / .cpp
│   ├── common/                  # 公共工具
│   │   ├── Logger.h / .cpp
│   │   ├── ThreadPool.h
│   │   ├── Config.h / .cpp
│   │   └── Exceptions.h
│   └── visualization/           # 可视化封装
│       └── VtkWidget.h / .cpp
├── algorithm/                   # MATLAB 源算法（只读参考）
├── config/                      # 运行时配置 JSON 文件
│   └── default_params.json
├── data/                        # 测试数据（.gitignore 忽略）
├── logs/                        # 运行日志（.gitignore 忽略）
├── build/                       # 编译输出（.gitignore 忽略）
├── test/                        # 单元测试代码
├── docs/                        # 项目文档（本目录）
├── twoProjector.vcxproj         # VS2019 项目文件
├── twoProjector.sln             # VS2019 解决方案文件
├── .gitignore
└── README.md
```

### 参数配置文件格式（JSON）

```json
{
  "reconstruction": {
    "decode_type": "multi_freq",
    "frequencies": [64, 56, 63],
    "phase_steps": [8, 8, 8],
    "modulation_threshold": 0.7,
    "depth_min_mm": 50.0,
    "depth_max_mm": 210.0,
    "phase_diff_threshold": 0.04,
    "epi_threshold_px": 1.5,
    "centroid_threshold": 0.05,
    "t_max": 10.0,
    "epsilon": 0.001,
    "fsnr_high": 0.30,
    "fsnr_low": 0.15,
    "mod_high": 30.0,
    "mod_low": 10.0
  },
  "paths": {
    "calib_pro1": "",
    "calib_pro2": "",
    "scan_folder": "",
    "epipolar_file": "",
    "save_path": ""
  }
}
```

---

## 7. 异常体系

### 7.1 异常类层次

```
std::exception
└── AppException（基类）
    ├── CalibException       — 标定加载失败、参数非法
    ├── ImageLoadException   — 图像文件不存在、格式错误
    ├── PhaseDecodeException — 调制度过低、相位解包失败
    ├── EpipolarException    — 极线配准失败
    ├── ReconstructException — 点云为空、高度超范围
    ├── MeasureException     — 焊球未检测到、测量点无效
    └── IOException          — 文件读写失败
```

### 7.2 处理规则

| 异常类型 | 捕获层 | 用户提示 | 日志级别 | 程序行为 |
|---------|--------|---------|---------|---------|
| `CalibException` | `MainWindow` | QMessageBox 弹窗 | ERROR | 中止重建，等待重新加载 |
| `ImageLoadException` | `ReconstructionPipeline` | 日志 + 进度条红色 | ERROR | 中止当前帧，可重试 |
| `PhaseDecodeException` | `PhaseDecoder` | 日志警告文本 | WARN | 跳过失败像素，继续 |
| `ReconstructException` | `MainWindow` | QMessageBox 弹窗 | ERROR | 中止，保留已有点云 |
| `MeasureException` | `BGAMeasurePipeline` | `bgaDataText` 提示 | WARN | 跳过无效点，继续测量 |
| 所有未知异常 | `main()` try/catch | 弹窗 + 日志 | FATAL | 安全退出 |

### 7.3 日志格式

```
[2026-03-07 22:00:00.123] [INFO]  ReconstructionPipeline::run — 开始重建，Pro1+Pro2
[2026-03-07 22:00:01.456] [WARN]  PhaseDecoder::unwrap — 低调制度像素数: 1234
[2026-03-07 22:00:02.789] [ERROR] EpipolarException — 极线偏差超过阈值: 3.2 px
```

日志文件路径：`logs/app_YYYYMMDD.log`，超过 10 MB 自动分割。

---

## 8. Git 规范

### 8.1 初始化命令（首次执行）

```bash
echo "# graduateSoftware" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/Touda8/graduateSoftware.git
git push -u origin main
```

### 8.2 .gitignore 必须包含

```gitignore
# 数据与日志
data/
logs/

# 编译输出
build/
.dist/
*.obj
*.exe
*.pdb
*.ilk
*.exp
*.lib

# VS 私有文件
.vs/
*.suo
*.user
*.VC.db
*.VC.opendb
x64/
Debug/
Release/

# Qt 生成文件
ui_*.h
moc_*.cpp

# 配置中的路径（用 default 替代）
config/user_*.json
```

### 8.3 Commit 规范

```
feat(reconstruction): 实现三频八步相位解码
fix(bga): 修正焊球定位在边缘失效问题
refactor(threadpool): 改用 QThreadPool 替代裸线程
docs: 更新 agent.md 中 Agent4 工程配置说明
```
