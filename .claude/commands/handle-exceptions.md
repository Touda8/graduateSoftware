---
description: "异常防护师：实现 Exceptions.h 异常层次、Logger 单例，覆盖 E01~E10 全部异常场景"
---

# 异常防护师 — Phase 3

**编号**：Agent-5  **阶段**：Phase 3（与测试验收师并行，依赖 Phase 2 完成）

## 前提检查（不满足则停止并报告）

使用 `Glob` / `Read` 验证：
- `src/reconstruction/PhaseDecoder.h` 存在
- `src/measurement/BGADetector.h` 存在
- `src/common/` 目录存在

## 工具使用

| 任务 | 工具 |
|------|------|
| 读取现有算法头文件 | `Read` |
| 查找函数位置 | `Grep`, `Glob` |
| 创建异常/Logger 骨架 | `Write`（≤ 30 行） |
| 逐类/逐函数插入异常处理 | `Edit`（每次 ≤ 50 行） |
| 验证覆盖完整性 | `Read` |

## 异常类层次（固定，不得在此之外新增根类）

```
AppException（基类）
├── CalibException          E01 标定文件相关
├── ImageLoadException      E02 图像读取 / 数量不足
├── PhaseDecodeException    E03/E04 相位解码异常
├── EpipolarException       E05/E08 极线配准失败
├── ReconstructException    E06/E09 重建失败 / 点云为空
├── MeasureException        E07 BGA 焊球未检测到
└── IOException             E10 文件读写失败
```

## 必须创建的文件

### src/common/Exceptions.h

```
Step 1  Write → #pragma once + #include <stdexcept>（≤ 10 行）
Step 2  Edit  → 插入 AppException 基类（≤ 20 行）
Step 3  Edit  → 插入 TP_DEFINE_EX 宏（≤ 10 行）
Step 4  Edit  → 逐个插入 7 个派生异常类（每类一次 Edit，每个 ≤ 15 行）
Step 5  Read  → 验证层次结构完整
```

### src/common/Logger.h

```
Step 1  Write → #pragma once + 枚举声明骨架（≤ 15 行）
Step 2  Edit  → 插入 Logger 单例类声明（instance()、log() 方法，≤ 30 行）
Step 3  Read  → 验证
```

### src/common/Logger.cpp

```
Step 1  Write → #include "Logger.h" + 命名空间（≤ 10 行）
Step 2  Edit  → 插入静态实例定义（≤ 10 行）
Step 3  Edit  → 插入 log() 实现（写文件 + 写 stderr，线程安全，≤ 40 行）
Step 4  Read  → 验证
```

Logger 规范：
- 日志文件：`logs/twoProjector_YYYYMMDD.log`，每天新文件，旧文件保留 7 天
- 格式：`[HH:mm:ss.zzz][LEVEL][Module] Message`
- 内部使用 `std::mutex` 保证线程安全

## 覆盖目标（E01~E10，按顺序逐条完成）

| 编号 | 位置 | 检查动作 |
|------|------|---------|
| E01 | `CalibLoader::load()` | `ifstream` 失败 → `CalibException` |
| E02 | `PhaseDecoder::setImages()` | 图像数量 < N×3 → `ImageLoadException` |
| E03 | `PhaseDecoder::calcFringeModulation()` | >80% 像素低调制度 → `WARN` 日志，继续 |
| E04 | `PhaseDecoder::unwrapPhase()` | 展开失败像素用 `NaN` 填充并记录比例 |
| E05 | `ReconstructionPipeline::correct()` | 极线偏差均值 > t → `EpipolarException` |
| E06 | `ReconstructionPipeline::buildCloud()` | `cloud->size() < 100` → `ReconstructException` |
| E07 | `BGADetector::detect2DBalls()` | `circles.empty()` → `MeasureException` |
| E08 | `BGADetector::matchBalls()` | 匹配率 < 30% → `EpipolarException` |
| E09 | `PointCloudFilter::SOR()` | Z 超量程 → 过滤 + 日志，不抛异常 |
| E10 | `PointCloudIO::save()` | `create_directories` 失败 → `IOException` |

**每条的操作流程**：
```
Step A  Read  → 定位目标函数
Step B  Edit  → 插入异常检查代码（≤ 30 行）
Step C  Read  → 验证插入正确
```

## 工作线程异常包装模板

```cpp
pool_.enqueue([this]() {
    try {
        /* 算法逻辑 */
    } catch (const AppException& e) {
        Logger::instance().log(Logger::ERROR, e.module(), e.what());
        emit finished(false, QString::fromStdString(e.what()));
    } catch (const std::exception& e) {
        emit finished(false, QString::fromStdString(e.what()));
    }
});
```

## UI 槽函数异常捕获模板

```cpp
void MainWindow::onStartRebuild() {
    try {
        pipeline_->runAsync(...);
    } catch (const CalibException& e) {
        QMessageBox::critical(this, "标定错误", e.what());
        setState(State::IDLE);
    } catch (const AppException& e) {
        appendLog(QString("[ERROR] %1").arg(e.what()));
        setState(State::READY);
    } catch (const std::exception& e) {
        appendLog(QString("[FATAL] 未知错误: %1").arg(e.what()));
    }
}
```

## 每次新增函数的异常检查清单

- [ ] 输入 `cv::Mat` 是否检查了 `.empty()`
- [ ] 图像尺寸是否与预期一致（rows、cols、type）
- [ ] 调制度是否发出 WARN 日志
- [ ] 极线配准结果是否在合理范围
- [ ] 点云输出 `size() > 100`
- [ ] 保存路径是否通过 `std::filesystem::create_directories` 保护

## 完成标志

- `src/common/Exceptions.h` 存在，含 7 个派生异常类
- `src/common/Logger.h` / `.cpp` 存在
- E01~E10 全部覆盖（使用 `Grep` 验证每条 catch/throw）

## 禁止

- 禁止 `catch(...)` 吞掉异常不处理
- 禁止在析构函数中抛出异常（加 `noexcept`）
- 禁止在 `Logger` 写入逻辑中再次抛出（避免循环异常）
- 禁止一次性为整个文件补充异常处理（按 E01 → E10 顺序逐条完成）
- 禁止在算法层使用 `std::cout` / `printf` / `qDebug()`
