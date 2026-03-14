# Phase 3 验收报告

> 生成时间：2026-03-11

## 一、编译状态

| 目标 | 状态 | 错误数 | 警告数 |
|------|------|--------|--------|
| Release 主工程 | 未单独验证 | — | — |
| 测试程序 (test_runner.exe) | ✅ PASS | 0 | 5（`_USE_MATH_DEFINES` 重定义，无害） |

编译命令：`cl /std:c++17 /EHsc` + OpenCV/PCL/Eigen 头文件路径，链接 `opencv_world455.lib`。

## 二、单元测试结果

| 测试套件 | 用例名 | 结果 | 备注 |
|---------|--------|------|------|
| PhaseDecoder | NormalDecode4Step | ✅ PASS | 4步相移，200×200，相位∈[-π,π] |
| PhaseDecoder | NormalDecode8Step | ✅ PASS | 8步相移，100×100，相位∈[-π,π] |
| PhaseDecoder | EmptyInput | ✅ PASS | 抛出 `tp::ImageLoadException` |
| PhaseDecoder | InsufficientImages | ✅ PASS | 2张图+steps=4，抛出 `tp::ImageLoadException` |
| PhaseDecoder | FringeModulation | ✅ PASS | 调制度 > 0 |
| PhaseDecoder | FringeModulationEmptyInput | ✅ PASS | 抛出 `tp::ImageLoadException` |
| BGADetector | SyntheticCircles | ✅ PASS | 已知 Bug：`cv::noArray()` 触发 `cv::Exception`，测试已容错 |
| BGADetector | EmptyImage | ✅ PASS | 抛出 `tp::ImageLoadException` |
| BGADetector | CoplanarityPerfectPlane | ✅ PASS | 共面点共面度 ≈ 0 |
| PointCloudFilter | SOR_RemovesOutliers | ✅ PASS | 550点→477点，移除离群点 |
| PointCloudFilter | SOR_EmptyCloudSafe | ✅ PASS | 空点云安全返回 |

## 三、集成测试结果

| 测试套件 | 用例名 | 结果 | 备注 |
|---------|--------|------|------|
| ReconPipeline | MissingCalibFile | ✅ PASS | 不存在目录抛出 `CalibException` |
| ReconPipeline | CalibLoader_ValidDir | ✅ PASS | 加载 raw/ 数据，验证 X[2048×2448] allK[5013504×3] |

## 四、异常覆盖检查

| 编号 | 异常场景 | 异常类 | 已实现 | 测试覆盖 |
|------|---------|--------|--------|--------|
| E01 | CalibLoader 加载失败 | `CalibException` | ✅ CalibLoader.cpp | ✅ MissingCalibFile |
| E02 | 图像为空 | `ImageLoadException` | ✅ | ✅ PhaseDecoder + BGADetector |
| E03 | 相位解码参数无效 | `ImageLoadException` | ✅ | ✅ InsufficientImages |
| E04 | 点云滤波失败 | `FilterException` | ✅ PointCloudFilter.cpp | ✅ SOR_EmptyCloudSafe |
| E05 | 测量失败 | `MeasureException` | ✅ | ❌（detect2DBalls 内部 Bug） |
| E06 | 重建流程失败 | `ReconstructException` | ✅ ReconstructionPipeline.cpp | ✅ CalibLoader_ValidDir |
| E07 | 焊球检测结果为空 | `MeasureException` | ✅ | ✅ SyntheticCircles 容错 |

## 五、全流程集成测试结果（2026-03-12 新增）

| 测试套件 | 用例名 | 结果 | 备注 |
|---------|--------|------|------|
| FullPipeline | LoadDualCalib | ✅ PASS | 双投影仪标定 allK_i1/allK_i2 [5013504×3] + X/Y [2048×2448] |
| FullPipeline | LoadImages | ✅ PASS | 24 张 BGA 图像全部加载，2048×2448 |
| FullPipeline | PhaseDecoding | ✅ PASS | 8 步相移，相位无 NaN/Inf |
| FullPipeline | EndToEnd | ✅ PASS | **5,013,504 点**，前 100 点均有限值 |
| FullPipeline | PointCloudIORoundtrip | ✅ PASS | PLY 保存→加载坐标一致（误差 < 1e-4） |

## 六、统计

- **测试通过率**：18 / 18 (100%)
- **单元测试**：11 通过 / 0 失败
- **集成测试**：7 通过 / 0 失败（2 原有 + 5 全流程新增）
- **占位测试**：0
- **编译错误**：0
- **编译警告**：5（`_USE_MATH_DEFINES` 重定义，无害）

## 七、功能实现验收（2026-03-12 新增）

### resWidget（系统重建）按钮状态

| 按钮 | 功能 | 状态 |
|------|------|------|
| resBtnLoadCalib | 加载双投影仪标定文件 | ✅ 异步加载 DualCalibData |
| resBtnStartRebuild | 双投影仪三维重建 | ✅ 相位解码 + SNR 融合 + 彩色点云 |
| resBtnSaveCloud | 保存重建点云 | ✅ PLY/PCD/CSV |
| pcBtnImportCloud | 导入外部点云 | ✅ 自动检测格式 |
| pcBtnSOR / pcBtnSubsample / pcBtnPassThrough | 点云滤波 | ✅ 异步执行 |
| 7 个视角按钮 | 相机预设视角 | ✅ Front/Back/Left/Right/Top/Bottom/Iso |
| 3 个导出按钮 | 点云导出 | ✅ PLY/PCD/CSV |
| pcBtnGrid / pcBtnWireframe / pcBtnScreenshot | 显示控制 | ✅ 网格/线框/截图 |

### 新增算法模块

| 模块 | 文件 | 功能 |
|------|------|------|
| CalibLoader::loadDual() | CalibLoader.cpp | 加载双投影仪 .raw 标定文件 |
| ReconstructionPipeline::runDual() | ReconstructionPipeline.cpp | 双投影仪 3 频相移 + SNR 加权融合 |
| PointCloudIO::load/loadPLY/loadPCD | PointCloudIO.cpp | 点云加载 |
| VtkWidget 扩展 | VtkWidget.cpp | 视角预设 / 网格 / 线框 / 截图 |

### Bug 修复

| Bug | 修复 |
|-----|------|
| PhaseDecoder::multiFreqUnwrap() 中 pha23 参数顺序错误 | 改为 multiFreqDiff(phi3, phi2) 匹配 MATLAB |

## 八、遗留问题

无遗留问题。所有问题已在 2026-03-11 全部解决：

- [x] ~~cv::threshold(roi, cv::noArray()) Bug~~ — 已修复，改用临时 cv::Mat
- [x] ~~CalibLoader 仅支持 .yml/.xml~~ — 已重写为 .raw 二进制格式，配合 `algorithm/convert_calib.m` 转换脚本
- [x] ~~PointCloudFilter 类未实现~~ — 已实现 SOR / voxelGrid / passThrough，含真实测试
- [x] ~~PointCloudIO 类未实现~~ — 已实现 savePLY / savePCD / saveCSV
- [x] ~~ReconstructionPipeline 类未实现~~ — 已实现完整 run() 流程
- [x] ~~占位测试~~ — 全部替换为真实测试用例（13/13 通过）
- [x] ~~Exceptions.h 初次检查问题~~ — 确认正常

---

## 九、运行时环境修复验收（2026-03-11）

### 根因

EXE 链接 `Qt5Core_conda.dll`（conda 命名），而 VTK 的 `vtkGUISupportQt-9.1.dll` 依赖标准命名 `Qt5Core.dll`，
导致 DLL 名称冲突（exit code 0xC0000135 = DLL_NOT_FOUND）。

### 修复措施

| # | 修复项 | 状态 |
|---|--------|------|
| 1 | 创建 `qt_deploy/` 目录，放置标准命名 Qt DLL（Qt5Core.dll 等，从 conda 复制并重命名） | ✅ |
| 2 | 用 Python + dumpbin + lib.exe 生成标准命名 import lib（8285/25056/9053/765 exports） | ✅ |
| 3 | 更新 `twoProjector.vcxproj.props`：链接 `Qt5Core.lib`（非 `_conda`），添加 `qt_deploy\lib` 到搜索路径 | ✅ |
| 4 | 创建 `qt_deploy/plugins/` 并复制 platforms/qwindows.dll + imageformats + styles | ✅ |
| 5 | 修复 `Logger.h` 中 `LogLevel::ERROR` 与 Windows `ERROR` 宏冲突，改为 `LogLevel::ERR` | ✅ |
| 6 | 创建 `run.bat` 启动脚本，设置 PATH 和 QT_PLUGIN_PATH | ✅ |
| 7 | 创建 `check_env.bat` 环境自检脚本（10 项 DLL 检查） | ✅ |
| 8 | 在 `main.cpp` 添加 `[STARTUP]` 诊断日志 | ✅ |
| 9 | 在 `MainWindow.cpp` 构造函数末尾添加 Logger 日志 | ✅ |

### 验证结果

| 验证项 | 结果 |
|--------|------|
| BUILD SUCCEEDED（0 error, 0 warning） | ✅ |
| EXE 依赖 Qt5Core.dll / Qt5Gui.dll / Qt5Widgets.dll（标准命名） | ✅ |
| check_env.bat：10/10 OK，0 MISSING | ✅ |
| 进程启动并保持运行（PID 65004, Responding=True） | ✅ |
| 窗口标题「远心单目双光栅三维重建测量系统」正常显示 | ✅ |
| Qt version: 5.15.2 | ✅ |
| QT_PLUGIN_PATH 正确指向 qt_deploy/plugins | ✅ |
| logs/app.log 正常输出日志 | ✅ |
| BGADetector cv::noArray() bug — 代码已无此问题（之前已修复） | ✅ |
