---
applyTo: "src/common/**"
---

# 异常分析智能体指令

> 适用范围：`src/common/`（Exceptions.h、Logger.h/.cpp）以及所有需要异常覆盖的函数

## 异常类层次（固定，不得在此之外新增异常根类）

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

新场景必须优先归入现有分类；确实无法归入时，才可在 `AppException` 下新增子类（需附理由注释）。

## 单目双投影专属异常场景（E01~E10）

| 编号 | 触发条件 | 异常类 | 处理策略 |
|------|---------|--------|---------|
| E01 | `.raw` 标定文件解析失败 | `CalibException` | 弹窗提示，中止 |
| E02 | 扫描图像数 < N×3 | `ImageLoadException` | 弹窗提示，中止 |
| E03 | >80% 像素调制度低于阈值 | `PhaseDecodeException` | `WARN` 日志，继续 |
| E04 | 三频相位矛盾无法展开 | `PhaseDecodeException` | 跳过像素，`INFO` 日志 |
| E05 | 极线偏差超过 t_max | `EpipolarException` | 日志 + 降级单投影 |
| E06 | 有效点云 < 100 个点 | `ReconstructException` | 弹窗，禁止保存 |
| E07 | Hough 焊球检测结果为空 | `MeasureException` | bgaDataText 友好提示 |
| E08 | 双投影匹配率 < 30% | `EpipolarException` | 弹窗，提示检查外参 |
| E09 | 高度超出 [depthMin, depthMax] | `ReconstructException`（不抛出） | 过滤超范围点，日志 |
| E10 | 保存路径无法创建 | `IOException` | 弹窗，要求重新选择路径 |

## 每次新增算法函数的异常检查清单

- [ ] 输入 `cv::Mat` 是否检查了 `.empty()`
- [ ] 图像尺寸是否与预期一致（`rows`、`cols`、`type`）
- [ ] 调制度是否发出 WARN 日志（不一定抛出）
- [ ] 极线配准结果是否在合理范围
- [ ] 点云输出 `size() > 100`
- [ ] 保存路径是否通过 `std::filesystem::create_directories` 保护

## Logger 使用规范

```cpp
// 获取单例
Logger::instance().log(Logger::INFO,  "PhaseDecoder", "三频解码开始");
Logger::instance().log(Logger::WARN,  "PhaseDecoder", "调制度过低，跳过部分像素");
Logger::instance().log(Logger::ERROR, "ReconPipeline", e.what());
```

- 禁止在算法层使用 `std::cout` / `printf` / `qDebug()`
- 日志文件：`logs/twoProjector_YYYYMMDD.log`，每天新文件，旧文件保留 7 天
- 日志格式：`[HH:mm:ss.zzz][LEVEL][Module] Message`
- 日志写入必须线程安全（内部使用 `std::mutex`）

## UI 槽函数异常捕获模板

```cpp
void MainWindow::onStartRebuild() {
    try {
        pipeline_->runAsync(ui->resEditScanFolder->text().toStdString());
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

## 工作线程异常捕获模板

```cpp
// 在 ThreadPool 任务 lambda 中
[this]() {
    try {
        // ... 算法逻辑
    } catch (const AppException& e) {
        emit logMessage(QtMsgType::QtCriticalMsg, e.what());
        emit finished(false, e.what());
    } catch (const std::exception& e) {
        emit finished(false, QString::fromStdString(e.what()));
    }
}
```

## 禁止事项

- 禁止 `catch(...)` 吞掉异常不处理
- 禁止在析构函数中抛出异常（加 `noexcept`）
- 禁止在 `Logger` 的写入逻辑中再次抛出（避免循环异常）
- 禁止仅 `catch` 而不记录日志（无声失败比崩溃更难调试）
