# 禁区控件列表

> capWidget（图像采集 Tab）及其所有子控件为**绝对禁区**，任何 Agent 不得修改。

## 一、capWidget 子控件清单

| objectName | class | 父控件 | 禁止原因 |
|-----------|-------|--------|---------|
| `capWidget` | QWidget | tabWidget | 图像采集 Tab 根控件（P01 禁区） |
| `capMainLayout` | QHBoxLayout | capWidget | capWidget 主布局 |
| `capLeftLayout` | QVBoxLayout | capMainLayout | 左侧控件区布局 |
| `capRightLayout` | QVBoxLayout | capMainLayout | 右侧显示区布局 |
| `capConnGroupBox` | QGroupBox | capLeftLayout | 连接状态分组 |
| `capConnLayout` | QVBoxLayout | capConnGroupBox | 连接状态布局 |
| `capTreeWidget` | QTreeWidget | capConnGroupBox | 设备连接树（相机+投影仪） |
| `capParamGroupBox` | QGroupBox | capLeftLayout | 参数设置分组 |
| `capParamGrid` | QGridLayout | capParamGroupBox | 参数设置网格布局 |
| `capLblTrigMode` | QLabel | capParamGroupBox | "触发模式"标签 |
| `capComboTrigMode` | QComboBox | capParamGroupBox | 触发模式选择（On/Off） |
| `capLblTrigSrc` | QLabel | capParamGroupBox | "触发源"标签 |
| `capComboTrigSrc` | QComboBox | capParamGroupBox | 触发源选择（Line0/Software/Line1） |
| `capLblExposure` | QLabel | capParamGroupBox | "曝光时间"标签 |
| `capEditExposure` | QLineEdit | capParamGroupBox | 曝光时间输入框 |
| `capLblFramerate` | QLabel | capParamGroupBox | "采集帧率"标签 |
| `capEditFramerate` | QLineEdit | capParamGroupBox | 采集帧率输入框 |
| `capLblSN` | QLabel | capParamGroupBox | "相机SN码"标签 |
| `capEditSN` | QLineEdit | capParamGroupBox | 相机序列号输入框 |
| `capBtnSoftTrigger` | QPushButton | capParamGroupBox | "单帧采集"按钮 |
| `capFileGroupBox` | QGroupBox | capLeftLayout | 文件设置分组 |
| `capFileLayout` | QVBoxLayout | capFileGroupBox | 文件设置布局 |
| `capFileRow` | QHBoxLayout | capFileGroupBox | 文件选择行布局 |
| `capLblFolder` | QLabel | capFileGroupBox | "采集图像文件夹"标签 |
| `capEditFolder` | QLineEdit | capFileGroupBox | 文件夹路径输入框 |
| `capBtnBrowse` | QPushButton | capFileGroupBox | "浏览"按钮 |
| `capScanGroupBox` | QGroupBox | capLeftLayout | 扫描采集分组 |
| `capScanLayout` | QVBoxLayout | capScanGroupBox | 扫描采集布局 |
| `capStepRow` | QHBoxLayout | capScanGroupBox | 相移步数选择行 |
| `capLblStep` | QLabel | capScanGroupBox | "相移步数"标签 |
| `capComboPhaseStep` | QComboBox | capScanGroupBox | 相移步数选择（四步/八步） |
| `capTrigRow` | QHBoxLayout | capScanGroupBox | 触发按钮行 |
| `capBtnPro1Trig` | QPushButton | capScanGroupBox | "Pro1触发"按钮 |
| `capBtnPro2Trig` | QPushButton | capScanGroupBox | "Pro2触发"按钮 |
| `capBtnContinuous` | QPushButton | capScanGroupBox | "连续采集"按钮 |
| `capLogGroupBox` | QGroupBox | capLeftLayout | 采集日志分组 |
| `capLogLayout` | QVBoxLayout | capLogGroupBox | 采集日志布局 |
| `capLogText` | QTextEdit | capLogGroupBox | 采集日志文本框 |
| `capTitleLabel` | QLabel | capRightLayout | "采集界面"标题标签 |
| `capShowLabel` | QLabel | capRightLayout | 条纹图像显示区 |

## 二、保护文件清单

| 文件/目录 | 禁止操作 | 原因 |
|----------|---------|------|
| `ui_twoProjector.h` | 禁止手动编辑 | uic 自动生成，每次构建覆盖（P05） |
| `F:\project\Envlib\` 及所有子目录 | 禁止修改、删除任何文件 | 共享第三方库环境（P02） |
| `twoProjector.ui` 中 `capWidget` 节点 | 禁止增删改任何属性 | 图像采集功能已完成验收（P01） |
| `algorithm/` 目录所有 `.m` 文件 | 只读参考，禁止修改 | MATLAB 算法原始代码，仅供翻译参照 |
