# UI-算法绑定表

> 系统重建（resWidget）、BGA 共面度测量（bgaWidget）和 QFP 共面度测量（qfpWidget）中所有可操作控件的信号-槽-算法绑定关系。

## resWidget — 系统重建

### 文件设置按钮

| objectName | class | 信号 | MainWindow 槽函数 | 调用的算法函数 |
|-----------|-------|------|------------------|--------------|
| `resBtnCalPro1Browse` | QPushButton | `clicked()` | `onResBtnCalPro1BrowseClicked()` | QFileDialog（更新 resEditCalPro1） |
| `resBtnCalPro2Browse` | QPushButton | `clicked()` | `onResBtnCalPro2BrowseClicked()` | QFileDialog（更新 resEditCalPro2） |
| `resBtnScanBrowse` | QPushButton | `clicked()` | `onResBtnScanBrowseClicked()` | QFileDialog（更新 resEditScanFolder） |
| `resBtnEpiBrowse` | QPushButton | `clicked()` | `onResBtnEpiBrowseClicked()` | QFileDialog（更新 resEditEpiFile） |
| `resBtnSavePathBrowse` | QPushButton | `clicked()` | `onResBtnSavePathBrowseClicked()` | QFileDialog（更新 resEditSavePath） |
| `resBtnTogglePanel` | QPushButton | `clicked()` | `onResBtnTogglePanelClicked()` | UI 逻辑：切换 resLeftWidget 可见性 |

### 重建控制按钮

| objectName | class | 信号 | MainWindow 槽函数 | 调用的算法函数 |
|-----------|-------|------|------------------|--------------|
| `resBtnLoadCalib` | QPushButton | `clicked()` | `onResBtnLoadCalibClicked()` | `CalibLoader::load()` |
| `resBtnStartRebuild` | QPushButton | `clicked()` | `onResBtnStartRebuildClicked()` | `ReconstructionPipeline::runAsync()` |
| `resBtnSaveCloud` | QPushButton | `clicked()` | `onResBtnSaveCloudClicked()` | `PointCloudIO::save()` |

### 重建参数控件

| objectName | class | 信号 | MainWindow 槽函数 | 调用的算法函数 |
|-----------|-------|------|------------------|--------------|
| `resComboDecodeType` | QComboBox | `currentIndexChanged(int)` | `onResComboDecodeTypeChanged(int)` | 更新 ReconParams::decodeType |
| `resCheckOrdered` | QCheckBox | `stateChanged(int)` | `onResCheckOrderedChanged(int)` | 更新 ReconParams::orderedCloud |
| `resSpinFreq1` | QSpinBox | `valueChanged(int)` | `onResSpinFreq1Changed(int)` | 更新 ReconParams::freq1 |
| `resSpinFreq2` | QSpinBox | `valueChanged(int)` | `onResSpinFreq2Changed(int)` | 更新 ReconParams::freq2 |
| `resSpinFreq3` | QSpinBox | `valueChanged(int)` | `onResSpinFreq3Changed(int)` | 更新 ReconParams::freq3 |
| `resSpinShift1` | QSpinBox | `valueChanged(int)` | `onResSpinShift1Changed(int)` | 更新 ReconParams::shift1 |
| `resSpinShift2` | QSpinBox | `valueChanged(int)` | `onResSpinShift2Changed(int)` | 更新 ReconParams::shift2 |
| `resSpinShift3` | QSpinBox | `valueChanged(int)` | `onResSpinShift3Changed(int)` | 更新 ReconParams::shift3 |
| `resDblSpinModThresh` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinModThreshChanged(double)` | 更新 ReconParams::modThresh |
| `resCheckDepthRange` | QCheckBox | `stateChanged(int)` | `onResCheckDepthRangeChanged(int)` | 启用/禁用深度范围限制 |
| `resDblSpinEpiThresh` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinEpiThreshChanged(double)` | 更新 ReconParams::epiThresh |
| `resDblSpinCentroidThresh` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinCentroidThreshChanged(double)` | 更新 ReconParams::centroidThresh |
| `resDblSpinTmax` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinTmaxChanged(double)` | 更新 ReconParams::tMax |
| `resDblSpinNumStable` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinNumStableChanged(double)` | 更新 ReconParams::numStable |
| `resDblSpinFSNRHigh` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinFSNRHighChanged(double)` | 更新 ReconParams::fsnrHigh |
| `resDblSpinFSNRLow` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinFSNRLowChanged(double)` | 更新 ReconParams::fsnrLow |
| `resDblSpinModHigh` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinModHighChanged(double)` | 更新 ReconParams::modHigh |
| `resDblSpinModLow` | QDoubleSpinBox | `valueChanged(double)` | `onResDblSpinModLowChanged(double)` | 更新 ReconParams::modLow |
| `resComboSaveMode` | QComboBox | `currentIndexChanged(int)` | `onResComboSaveModeChanged(int)` | 切换单次/多次重建模式 |
| `resSpinRepeatCount` | QSpinBox | `valueChanged(int)` | `onResSpinRepeatCountChanged(int)` | 更新 ReconParams::repeatCount |
| `resCheckGrid` | QCheckBox | `stateChanged(int)` | `onResCheckGridChanged(int)` | VtkWidget 网格显示开关 |
| `resCheckWireframe` | QCheckBox | `stateChanged(int)` | `onResCheckWireframeChanged(int)` | VtkWidget 线框模式开关 |
| `resCheckInvert` | QCheckBox | `stateChanged(int)` | `onResCheckInvertChanged(int)` | VtkWidget 高度反转开关 |

### 点云处理按钮

| objectName | class | 信号 | MainWindow 槽函数 | 调用的算法函数 |
|-----------|-------|------|------------------|--------------|
| `pcBtnImportCloud` | QPushButton | `clicked()` | `onPcBtnImportCloudClicked()` | `PointCloudIO::load()` |
| `pcBtnCropROI` | QPushButton | `clicked()` | `onPcBtnCropROIClicked()` | `pcl::CropBox` |
| `pcBtnCropPlane` | QPushButton | `clicked()` | `onPcBtnCropPlaneClicked()` | `pcl::PlaneClipper3D` |
| `pcBtnSubsample` | QPushButton | `clicked()` | `onPcBtnSubsampleClicked()` | `PointCloudFilter::voxelGrid()` |
| `pcBtnMerge` | QPushButton | `clicked()` | `onPcBtnMergeClicked()` | `PointCloudMerger::merge()` |
| `pcBtnDuplicate` | QPushButton | `clicked()` | `onPcBtnDuplicateClicked()` | `pcl::UniformSampling` |
| `pcBtnSOR` | QPushButton | `clicked()` | `onPcBtnSORClicked()` | `PointCloudFilter::SOR()` |
| `pcBtnROR` | QPushButton | `clicked()` | `onPcBtnRORClicked()` | `PointCloudFilter::ROR()` |
| `pcBtnPassThrough` | QPushButton | `clicked()` | `onPcBtnPassThroughClicked()` | `PointCloudFilter::passThrough()` |
| `pcBtnGaussian` | QPushButton | `clicked()` | `onPcBtnGaussianClicked()` | `pcl::MovingLeastSquares` |
| `pcBtnFitPlane` | QPushButton | `clicked()` | `onPcBtnFitPlaneClicked()` | `GeometryAnalyzer::fitPlanePCA()` / RANSAC / LeastSquares |
| `pcBtnNormal` | QPushButton | `clicked()` | `onPcBtnNormalClicked()` | `pcl::NormalEstimationOMP` |
| `pcBtnCurvature` | QPushButton | `clicked()` | `onPcBtnCurvatureClicked()` | `pcl::PrincipalCurvaturesEstimation` |
| `pcBtnRoughness` | QPushButton | `clicked()` | `onPcBtnRoughnessClicked()` | `GeometryAnalyzer::calcRoughness()` |
| `pcBtnICP` | QPushButton | `clicked()` | `onPcBtnICPClicked()` | `pcl::IterativeClosestPoint` |
| `pcBtnDistance` | QPushButton | `clicked()` | `onPcBtnDistanceClicked()` | `GeometryAnalyzer::distanceToPlane()` |
| `pcBtnPCAAlign` | QPushButton | `clicked()` | `onPcBtnPCAAlignClicked()` | `pcl::PCA` |
| `pcBtnCenter` | QPushButton | `clicked()` | `onPcBtnCenterClicked()` | 平移质心至原点 |
| `pcBtnLevelPlane` | QPushButton | `clicked()` | `onPcBtnLevelPlaneClicked()` | 拟合平面旋转至水平 |
| `pcBtnApplyTransform` | QPushButton | `clicked()` | `onPcBtnApplyTransformClicked()` | `pcl::transformPointCloud` |
| `pcBtnViewFront` | QPushButton | `clicked()` | `onPcBtnViewFrontClicked()` | VtkWidget camera preset |
| `pcBtnViewBack` | QPushButton | `clicked()` | `onPcBtnViewBackClicked()` | VtkWidget camera preset |
| `pcBtnViewLeft` | QPushButton | `clicked()` | `onPcBtnViewLeftClicked()` | VtkWidget camera preset |
| `pcBtnViewRight` | QPushButton | `clicked()` | `onPcBtnViewRightClicked()` | VtkWidget camera preset |
| `pcBtnViewTop` | QPushButton | `clicked()` | `onPcBtnViewTopClicked()` | VtkWidget camera preset |
| `pcBtnViewBottom` | QPushButton | `clicked()` | `onPcBtnViewBottomClicked()` | VtkWidget camera preset |
| `pcBtnViewIsometric` | QPushButton | `clicked()` | `onPcBtnViewIsometricClicked()` | VtkWidget camera preset |
| `pcBtnViewReset` | QPushButton | `clicked()` | `onPcBtnViewResetClicked()` | VtkWidget camera reset |
| `pcBtnExportPLY` | QPushButton | `clicked()` | `onPcBtnExportPLYClicked()` | `PointCloudIO::save(PLY)` |
| `pcBtnExportPCD` | QPushButton | `clicked()` | `onPcBtnExportPCDClicked()` | `PointCloudIO::save(PCD)` |
| `pcBtnExportCSV` | QPushButton | `clicked()` | `onPcBtnExportCSVClicked()` | `PointCloudIO::save(CSV)` |
| `pcBtnScreenshot` | QPushButton | `clicked()` | `onPcBtnScreenshotClicked()` | `vtkWindowToImageFilter` |

### 点云参数控件

| objectName | class | 信号 | MainWindow 槽函数 | 调用的算法函数 |
|-----------|-------|------|------------------|--------------|
| `pcSpinVoxelSize` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinVoxelSizeChanged(double)` | 体素降采样参数 |
| `pcSpinSORK` | QSpinBox | `valueChanged(int)` | `onPcSpinSORKChanged(int)` | SOR K近邻数 |
| `pcSpinSORStd` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinSORStdChanged(double)` | SOR 标准差倍数 |
| `pcSpinRORRadius` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinRORRadiusChanged(double)` | ROR 搜索半径 |
| `pcSpinRORMin` | QSpinBox | `valueChanged(int)` | `onPcSpinRORMinChanged(int)` | ROR 最小邻域点数 |
| `pcComboPassAxis` | QComboBox | `currentIndexChanged(int)` | `onPcComboPassAxisChanged(int)` | 直通滤波坐标轴 |
| `pcComboFitMethod` | QComboBox | `currentIndexChanged(int)` | `onPcComboFitMethodChanged(int)` | 平面拟合方法选择 |
| `pcSpinNormalR` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinNormalRChanged(double)` | 法线估计半径 |
| `pcSpinRotX` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinRotXChanged(double)` | 旋转角 Rx |
| `pcSpinRotY` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinRotYChanged(double)` | 旋转角 Ry |
| `pcSpinRotZ` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinRotZChanged(double)` | 旋转角 Rz |
| `pcSpinTransX` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinTransXChanged(double)` | 平移量 Tx |
| `pcSpinTransY` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinTransYChanged(double)` | 平移量 Ty |
| `pcSpinTransZ` | QDoubleSpinBox | `valueChanged(double)` | `onPcSpinTransZChanged(double)` | 平移量 Tz |

## bgaWidget — BGA 共面度测量

| objectName | class | 信号 | MainWindow 槽函数 | 调用的算法函数 |
|-----------|-------|------|------------------|--------------|
| `bgaSpinCount` | QSpinBox | `valueChanged(int)` | `onBgaSpinCountChanged(int)` | 更新测量次数参数 |
| `bgaBtnStart` | QPushButton | `clicked()` | `onBgaBtnStartClicked()` | `BGAMeasurePipeline::runAsync()` |
| `bgaBtnPlot` | QPushButton | `clicked()` | `onBgaBtnPlotClicked()` | `BGAPlotter::plot()` |
| `bgaBtnBallShow` | QPushButton | `clicked()` | `onBgaBtnBallShowClicked()` | `BGADetector::detect2DBalls()` 叠加显示 |
| `bgaBtnImport` | QPushButton | `clicked()` | `onBgaBtnImportClicked()` | 文件对话框 + CSV 解析 |
| `bgaBtnSave` | QPushButton | `clicked()` | `onBgaBtnSaveClicked()` | 导出 CSV/TXT |

## qfpWidget — QFP 共面度测量

| objectName | class | 信号 | MainWindow 槽函数 | 调用的算法函数 |
|-----------|-------|------|------------------|--------------|
| `qfpSpinCount` | QSpinBox | `valueChanged(int)` | `onQfpSpinCountChanged(int)` | 更新测量次数参数 |
| `qfpBtnStart` | QPushButton | `clicked()` | `onQfpBtnStartClicked()` | `QFPMeasurePipeline::runAsync()` |
| `qfpBtnPlot` | QPushButton | `clicked()` | `onQfpBtnPlotClicked()` | `QFPPlotter::plot()` |
| `qfpBtnPinShow` | QPushButton | `clicked()` | `onQfpBtnPinShowClicked()` | `QFPSegmentor` 分割结果叠加显示 |
| `qfpBtnImport` | QPushButton | `clicked()` | `onQfpBtnImportClicked()` | 文件对话框 + CSV 解析 |
| `qfpBtnSave` | QPushButton | `clicked()` | `onQfpBtnSaveClicked()` | 导出 CSV/TXT |
