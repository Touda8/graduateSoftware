---
applyTo: "src/MainWindow*,src/visualization/**,src/common/Config*"
---

# UI 开发智能体指令

> 适用范围：`src/MainWindow.*`、`src/visualization/`、`src/common/Config.*`

## 核心禁区（P01）

以下内容**在任何情况下都不得修改**：
- `twoProjector.ui` 中 `capWidget` 及其所有直接 / 间接子控件
- `ui_twoProjector.h`（uic 自动生成，每次构建覆盖，禁止手动编辑）
- 任何以 `cap` 为前缀的控件的信号槽连接

## 控件命名前缀（已固定，不可更改）

| 前缀 | Tab | 开发状态 |
|------|-----|---------|
| `cap*` | 图像采集（Tab 0） | **禁区** |
| `res*` | 系统重建（Tab 1） | 重点开发 |
| `pc*` | 点云处理（Tab 1 子区域） | 重点开发 |
| `bga*` | BGA 共面度测量（Tab 2） | 重点开发 |

## 线程分离规则（P03 / P04）

```cpp
// ✅ 正确：跨线程更新进度条
connect(worker, &ReconstructionPipeline::progressUpdated,
        ui->resProgressBar, &QProgressBar::setValue,
        Qt::QueuedConnection);

// ❌ 严禁：工作线程中直接操作控件
ui->resProgressBar->setValue(50);   // 崩溃风险！
ui->resLogText->append("...");       // 崩溃风险！
```

- 计算线程通过 `emit signal(...)` 传递数据，UI 线程负责接收并渲染
- 即使是 `setText`、`append` 等"看似安全"的操作，也必须通过信号槽

## 信号槽声明规范

- 统一在 `.cpp` 中用 `connect()` 声明，**不使用** `SIGNAL`/`SLOT` 宏
- 优先使用函数指针形式（编译期检查）：

```cpp
connect(ui->resBtnStartRebuild, &QPushButton::clicked,
        this, &MainWindow::onStartRebuild);
```

## 状态机（系统重建 Tab）

```
[IDLE]
  → 加载标定文件 → [CALIB_LOADING] → 成功 → [READY]
                                    → 失败 → [IDLE] + 弹窗
  → 开始重建（仅 READY 可触发）→ [RECONSTRUCTING]
                                → 成功 → [DONE]（解锁"保存点云"）
                                → 失败 → [READY] + 日志
  → 保存点云（仅 DONE 可触发）→ 写文件 → [DONE]
```

所有按钮的 `setEnabled()` 随状态变化统一由 `MainWindow::setState(State s)` 管理。

## VTK 嵌入规范

- `resCloudLabel`（QLabel 占位符）在 `MainWindow` 构造时替换为 `QVTKOpenGLNativeWidget`
- VTK 渲染器、摄像机操作只能在 UI 主线程执行
- 点云高度映射 Jet 色表（低→蓝，高→红），通过 `vtkLookupTable` 实现
- 8 个视角按钮（front/back/left/right/top/bottom/iso/reset）通过 `vtkCamera::SetPosition` 实现

## 参数持久化（Config）

- 参数存储路径：`config/params.json`（应用启动时加载，关闭时保存）
- 以控件 `objectName` 为 key，控件当前值为 value
- `Config::save(QWidget* root)` / `Config::load(QWidget* root)` 统一处理所有子控件

```json
{
  "resSpinFreq1": 64,
  "resDblSpinModThresh": 0.7,
  "resEditSavePath": "D:/output"
}
```

## 日志追加格式

```cpp
// 必须添加时间戳；此函数通过 QueuedConnection 从工作线程触发
void MainWindow::appendLog(const QString& msg) {
    ui->resLogText->append(
        QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg);
}
```

## 进度条规范

- 重建开始：`resProgressBar->setValue(0)`
- 重建完成：`resProgressBar->setValue(100)`
- 中间阶段：按流水线步骤比例分配 0~100
