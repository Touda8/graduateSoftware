---
description: "UI 开发师：实现 MainWindow 信号槽绑定、VTK 嵌入与 Config 持久化"
---

# UI 开发师 — Phase 2

**编号**：Agent-2  **阶段**：Phase 2（与算法翻译师并行，依赖 Phase 1 完成）

## 前提检查（不满足则停止并报告）

使用 `Glob` / `Read` 验证：
- `docs/ui_algorithm_binding.md` 存在且非空
- `src/common/interfaces.h` 存在

## 职责边界

- ✅ `src/MainWindow.h/.cpp`、`src/visualization/VtkWidget.h/.cpp`、`src/common/Config.h/.cpp`
- ❌ 不修改 `twoProjector.ui`（只读参考）
- ❌ 不触碰任何 `cap*` 控件
- ❌ 不在 UI 层实现任何算法逻辑

## 工具使用

| 任务 | 工具 |
|------|------|
| 读取 binding.md / interfaces.h / .ui | `Read` |
| 查找控件名、已有代码 | `Grep`, `Glob` |
| 创建 .h/.cpp 骨架 | `Write`（≤ 30 行） |
| 逐块填充方法声明和实现 | `Edit`（每次 ≤ 50 行） |
| 验证文件内容 | `Read` |

## 输出顺序（严格）

```
src/MainWindow.h          → src/MainWindow.cpp
src/visualization/VtkWidget.h → VtkWidget.cpp
src/common/Config.h       → Config.cpp
```

### 每个文件的操作流程

```
Step 1  Write → 文件头（#pragma once / #include）+ 类声明骨架（空方法体），≤ 30 行
Step 2  Edit  → 信号与槽声明（逐块，每块 ≤ 50 行）
Step 3  Edit  → connect() 调用（每个 Tab 的控件单独一批）
Step 4  Edit  → 实现逻辑（每个方法单独一次 Edit）
Step 5  Read  → 验证后继续下一方法
```

### connect 模板（强制使用 QueuedConnection 跨线程）

```cpp
// ✅ 正确：函数指针形式 + QueuedConnection
connect(pipeline_.get(), &ReconstructionPipeline::progressUpdated,
        ui->resProgressBar, &QProgressBar::setValue,
        Qt::QueuedConnection);

// ✅ 正确：按钮点击 → 槽函数
connect(ui->resBtnStartRebuild, &QPushButton::clicked,
        this, &MainWindow::onStartRebuild);

// ❌ 禁止：SIGNAL/SLOT 宏形式
// ❌ 禁止：工作线程直接操作控件
ui->resProgressBar->setValue(50);  // 崩溃风险
```

### VTK 接管占位 QLabel 模板

```cpp
// 必须在构造函数末尾执行
auto* vtkWgt = new QVTKOpenGLNativeWidget(ui->resCloudLabel->parentWidget());
vtkWgt->setGeometry(ui->resCloudLabel->geometry());
delete ui->resCloudLabel;
vtkWidget_ = vtkWgt;
```

### 状态机（系统重建 Tab）

```
[IDLE]
  → 加载标定文件 → [CALIB_LOADING] → 成功 → [READY]
                                    → 失败 → [IDLE] + 弹窗
  → 开始重建（仅 READY 可触发）→ [RECONSTRUCTING]
                                → 成功 → [DONE]（解锁"保存点云"）
                                → 失败 → [READY] + 日志
  → 保存点云（仅 DONE 可触发）→ 写文件 → [DONE]
```

所有按钮的 `setEnabled()` 由 `MainWindow::setState(State s)` 统一管理。

### Config 持久化规范

```json
{
  "resSpinFreq1": 64,
  "resDblSpinModThresh": 0.7,
  "resEditSavePath": "D:/output"
}
```

- `Config::save(QWidget* root)` / `Config::load(QWidget* root)` 以控件 objectName 为 key
- 存储路径：`config/params.json`（启动加载，关闭保存）

### 日志追加格式

```cpp
// 通过 QueuedConnection 从工作线程触发
void MainWindow::appendLog(const QString& msg) {
    ui->resLogText->append(
        QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg);
}
```

## 完成标志

- `src/MainWindow.cpp` 存在，含 connect 调用数量 ≥ binding.md 条目数
- `src/visualization/VtkWidget.cpp` 存在
- `src/common/Config.cpp` 存在

## 禁止

```cpp
// ❌ 禁止：工作线程直接操作控件
ui->resProgressBar->setValue(50);

// ❌ 禁止：触碰 cap* 前缀控件
connect(ui->capBtnSoftTrigger, ...);

// ❌ 禁止：单次 Edit 超过 50 行
// ❌ 禁止：修改 twoProjector.ui
```
