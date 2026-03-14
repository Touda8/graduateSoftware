
# 任务：将 Project2 投影仪控制功能嫁接到 graduateSoftware

## 一、授权声明

用户已明确授权：本任务允许修改 twoProjector.ui 中 capWidget（Tab 0 图像采集）的 UI 内容
以及对应的后端逻辑代码。此授权仅针对本次投影仪控制功能集成任务，其余 P01~P06 约束继续生效。

---

## 二、项目背景

### 主项目
- 路径：f:\project\graduateSoftware\
- 语言：C++17 / Qt（Qt VS Tools）/ Visual Studio 2019 x64 v142
- 主窗口类：twoProjectorClass : QMainWindow
- 主 UI 文件：twoProjector.ui（禁止手动修改 ui_twoProjector.h）

### Project2（投影仪控制库，只读参考）
- 路径：f:\project\graduateSoftware\Project2\
- 核心文件：
  - Project2/JMTIController.h — 多设备 Cypress USB-I2C 控制器类声明
  - Project2/JMTIController.cpp — 实现（含 GPIO 握手、I2C 读写、投影控制）
  - Project2/CyUSBSerial.h — Cypress USB-Serial API
  - Project2/cyusbserial.dll — 运行时 DLL（需复制到输出目录）
  - projector_dlpc_api/include/projectorDlpc34xx.h — TI 官方高层封装
  - projector_dlpc_api/src/projectorDlpc34xx.cpp — Flash 图案烧录与播放

### JMTIController 关键能力
- EnumerateDevices() — 枚举所有 Cypress I2C 接口设备
- ConnectDevice(int idx) / DisconnectDevice(int idx) — 设备连接管理
- GetDeviceList() — 返回所有设备描述字符串列表
- IsDeviceConnected(int idx) — 查询连接状态
- OpenTI(int idx) — 连接 + GPIO 握手（完整初始化序列）
- slotTriggerITBoard(int idx) — 触发内部图案序列（Flash 存储的条纹）
- slotTriggerStop(int idx) — 停止投影（进入待机模式）
- RunPatternSequence(int idx, bool runOnce) — Flash 图案序列播放
  - runOnce=true：单次播放
  - runOnce=false：循环播放
- StartTestPattern(int idx, DlpcTestPattern pattern, DlpcColor color) — 显示测试图案
  - 图案枚举：TP_SOLID_FIELD / TP_HORIZONTAL_LINES / TP_VERTICAL_LINES /
               TP_GRID / TP_CHECKERBOARD / TP_COLORBARS / TP_DIAGONAL_LINES
- SetOperatingMode(int idx, DlpcOperatingMode mode) — 切换工作模式
  - 枚举：MODE_EXTERNAL_VIDEO / MODE_TEST_PATTERN / MODE_INTERNAL_PATTERN / MODE_STANDBY
- SetLedEnable(int idx, bool r, bool g, bool b) — LED 使能
- ReadShortStatus(int idx, uint8_t* statusByte) — 读取设备状态字节

---

## 三、capWidget 现有控件（勿重复添加）

已有以下投影仪相关控件（objectName 以 cap 开头）：
- capTreeWidget — 设备树（已有相机行 + Pro1 行 + Pro2 行），maxHeight=140
- capBtnPro1Trig — "Pro1触发"（已有，保留）
- capBtnPro2Trig — "Pro2触发"（已有，保留）
- capBtnContinuous — "连续采集"（已有，保留）
- capBtnSoftTrigger — "单帧采集"（已有，保留）
- capScanGroupBox — 已有"扫描采集"组（capBtnPro1Trig / capBtnPro2Trig 在此组内）
- capLogText — 采集日志 QTextEdit（已有，保留并复用用于投影仪日志）

---

## 四、需要新增的 UI 控件（在 twoProjector.ui 中添加）

在 capWidget 左侧面板（capLeftLayout）中，于 capConnGroupBox（连接状态）与 capParamGroupBox（参数设置）之间插入新分组。
所有新增控件命名严格使用 cap 前缀。

### 4.1 capProjConnGroupBox — "投影仪连接"组

| objectName | 控件类型 | 说明 |
|---|---|---|
| capBtnProjScan | QPushButton | "扫描设备" |
| capLblPro1Status | QLabel | "Pro1: 未连接" 红色 |
| capBtnPro1Connect | QPushButton | "连接" maxWidth=60 |
| capBtnPro1Disconnect | QPushButton | "断开" maxWidth=60 enabled=false |
| capLblPro2Status | QLabel | "Pro2: 未连接" 红色 |
| capBtnPro2Connect | QPushButton | "连接" maxWidth=60 |
| capBtnPro2Disconnect | QPushButton | "断开" maxWidth=60 enabled=false |
| capBtnAllConnect | QPushButton | "全部连接" |
| capBtnAllDisconnect | QPushButton | "全部断开" enabled=false |

布局要求：
- capBtnProjScan 占一整行
- Pro1 状态标签 + 两个按钮 水平排列（HBoxLayout）
- Pro2 状态标签 + 两个按钮 水平排列（HBoxLayout）
- capBtnAllConnect / capBtnAllDisconnect 水平排列

### 4.2 capProjTestGroupBox — "投影测试"组

| objectName | 控件类型 | 说明 |
|---|---|---|
| capLblTestProjSel | QLabel | "目标投影仪:" |
| capComboProjTestSel | QComboBox | 条目: "Pro1" / "Pro2" / "全部" |
| capLblTestPattern | QLabel | "测试图案:" |
| capComboProjPattern | QComboBox | 条目: "实色-白" / "实色-黑" / "彩色条" / "水平线" / "垂直线" / "格栅" / "棋盘格" |
| capBtnProjShowTest | QPushButton | "显示测试图案" |
| capBtnProjStopTest | QPushButton | "停止测试图案" |

### 4.3 capPatternGroupBox — "条纹投射"组（扩充原 capScanGroupBox）

在原有 capBtnPro1Trig / capBtnPro2Trig 下方追加：

| objectName | 控件类型 | 说明 |
|---|---|---|
| capLblPatternMode | QLabel | "投射模式:" |
| capComboPatternMode | QComboBox | 条目: "单次投射" / "循环投射" |
| capLblPatternProjSel | QLabel | "投射投影仪:" |
| capComboPatternProjSel | QComboBox | 条目: "Pro1" / "Pro2" / "顺序(Pro1→Pro2)" / "全部同时" |
| capBtnPatternStart | QPushButton | "开始投射" |
| capBtnPatternStop | QPushButton | "停止投射" |
| capSpinPatternDelay | QSpinBox | 投影仪间切换延迟(ms) value=100 min=10 max=5000 |
| capLblPatternDelay | QLabel | "切换延迟(ms):" |

---

## 五、需要新增的后端类

### 5.1 ProjectorManager（新建）

- 头文件：src/common/ProjectorManager.h
- 实现：src/common/ProjectorManager.cpp
- 继承：QObject（支持信号槽）

#### 职责
封装 JMTIController，提供线程安全的投影仪控制接口，所有耗时操作在 QThread 中执行。

#### 接口设计

```cpp
#pragma once
#include <QObject>
#include <QThread>
#include <memory>
#include <vector>
#include <string>
#include "../../Project2/Project2/JMTIController.h"

class ProjectorManager : public QObject {
    Q_OBJECT
public:
    static constexpr int PRO1_IDX = 0;
    static constexpr int PRO2_IDX = 1;

    explicit ProjectorManager(QObject* parent = nullptr);
    ~ProjectorManager() override;

    // 同步查询（UI 线程安全，不耗时）
    bool isConnected(int projIdx) const;
    int  connectedCount() const;
    std::vector<std::string> getDeviceList() const;

public slots:
    void scanDevices();
    void connectDevice(int projIdx);
    void disconnectDevice(int projIdx);
    void connectAll();
    void disconnectAll();
    void showTestPattern(int projIdx, int patternCode, int colorCode);
    void stopTestPattern(int projIdx);
    void startPatternSequence(int projIdx, bool runOnce);
    void stopPatternSequence(int projIdx);
    void triggerSequential(bool runOnce, int delayMs);  // Pro1先后Pro2
    void triggerAll(bool runOnce);                       // 同时触发所有

signals:
    void deviceListUpdated(QStringList devices);
    void connectionStateChanged(int projIdx, bool connected);
    void statusMessage(QString msg);            // 追加到 capLogText
    void errorOccurred(int projIdx, QString err);
    void allConnected();
    void allDisconnected();

private:
    std::unique_ptr<JMTIController> controller_;
    mutable std::mutex mutex_;
    QThread* workerThread_;
};
```

#### 实现要求
- constructor：创建 workerThread_，将 this moveToThread(workerThread_)，启动线程
- 所有 slot 实现在工作线程运行（通过 Qt::QueuedConnection 从 UI 发起）
- 操作前先 lock mutex_
- 每个操作完成后发出对应信号，失败时发出 errorOccurred
- 析构时 workerThread_->quit() → wait() → 调用 controller_->DisconnectAllDevices()

### 5.2 twoProjector（主窗口）修改

在 twoProjector.h 中添加：
```cpp
#include "src/common/ProjectorManager.h"
private:
    ProjectorManager* projMgr_ = nullptr;
    void setupProjectorManager();
    void updateProStatusLabel(int projIdx, bool connected);
```

在 twoProjector.cpp 的构造函数（或 setupUi 之后）中调用 setupProjectorManager()。

---

## 六、信号槽绑定规范

所有从 UI 到 ProjectorManager 的连接必须使用 Qt::QueuedConnection（因 ProjectorManager 在工作线程）：

```cpp
// 扫描设备
connect(ui->capBtnProjScan, &QPushButton::clicked,
        projMgr_, &ProjectorManager::scanDevices,
        Qt::QueuedConnection);

// Pro1 连接/断开
connect(ui->capBtnPro1Connect, &QPushButton::clicked,
        this, [this]{ projMgr_->connectDevice(ProjectorManager::PRO1_IDX); },
        Qt::QueuedConnection);
connect(ui->capBtnPro1Disconnect, &QPushButton::clicked,
        this, [this]{ projMgr_->disconnectDevice(ProjectorManager::PRO1_IDX); },
        Qt::QueuedConnection);

// 状态更新回到 UI（QueuedConnection 自动跨线程）
connect(projMgr_, &ProjectorManager::connectionStateChanged,
        this, &twoProjectorClass::updateProStatusLabel,
        Qt::QueuedConnection);

connect(projMgr_, &ProjectorManager::statusMessage,
        ui->capLogText, &QTextEdit::append,
        Qt::QueuedConnection);

connect(projMgr_, &ProjectorManager::deviceListUpdated,
        this, [this](QStringList list) {
            // 更新 capTreeWidget 中 Pro1/Pro2 节点的 ToolTip 和文本
        }, Qt::QueuedConnection);

// 测试图案
connect(ui->capBtnProjShowTest, &QPushButton::clicked, this, [this] {
    int sel = ui->capComboProjTestSel->currentIndex();  // 0=Pro1,1=Pro2,2=All
    int pat = ui->capComboProjPattern->currentIndex();
    // 映射 patternCode 到 DlpcTestPattern，colorCode 到 DlpcColor
    if (sel == 0 || sel == 2)
        projMgr_->showTestPattern(0, pat, 7/*WHITE*/);
    if (sel == 1 || sel == 2)
        projMgr_->showTestPattern(1, pat, 7/*WHITE*/);
}, Qt::QueuedConnection);

// 条纹投射
connect(ui->capBtnPatternStart, &QPushButton::clicked, this, [this] {
    bool runOnce = (ui->capComboPatternMode->currentIndex() == 0);
    int sel = ui->capComboPatternProjSel->currentIndex();
    int delay = ui->capSpinPatternDelay->value();
    if (sel == 0) projMgr_->startPatternSequence(0, runOnce);
    else if (sel == 1) projMgr_->startPatternSequence(1, runOnce);
    else if (sel == 2) projMgr_->triggerSequential(runOnce, delay);
    else               projMgr_->triggerAll(runOnce);
}, Qt::QueuedConnection);
```

---

## 七、updateProStatusLabel 实现

```cpp
void twoProjectorClass::updateProStatusLabel(int projIdx, bool connected) {
    QLabel* lbl = (projIdx == 0) ? ui->capLblPro1Status : ui->capLblPro2Status;
    QPushButton* btnConn = (projIdx == 0) ? ui->capBtnPro1Connect : ui->capBtnPro2Connect;
    QPushButton* btnDisc = (projIdx == 0) ? ui->capBtnPro1Disconnect : ui->capBtnPro2Disconnect;

    QString name = QString("Pro%1").arg(projIdx + 1);
    if (connected) {
        lbl->setText(name + ": 已连接");
        lbl->setStyleSheet("color: green; font-weight: bold;");
        btnConn->setEnabled(false);
        btnDisc->setEnabled(true);
    } else {
        lbl->setText(name + ": 未连接");
        lbl->setStyleSheet("color: red;");
        btnConn->setEnabled(true);
        btnDisc->setEnabled(false);
    }

    // 同步更新 capTreeWidget 对应节点
    // Pro1 → topLevelItem(1), Pro2 → topLevelItem(2) (约定)
    QTreeWidgetItem* item = ui->capTreeWidget->topLevelItem(projIdx + 1);
    if (item) item->setText(1, connected ? "已连接" : "未连接");
}
```

---

## 八、capTreeWidget 自动状态更新策略

订阅 ProjectorManager::connectionStateChanged 信号，在 updateProStatusLabel 槽中同步更新
capTreeWidget 中对应节点（无需定时器轮询，纯事件驱动）。

在 scanDevices() 执行完毕后：
- 发出 deviceListUpdated(list) 信号
- UI 槽中将设备 ID/序列号回填到 capTreeWidget 对应节点的 ToolTip

---

## 九、DLL 部署要求

将 f:\project\graduateSoftware\Project2\Project2\cyusbserial.dll
复制到项目的 build\Release\ 和 build\Debug\ 输出目录。
在 .vcxproj 或 build.bat 中添加 xcopy 步骤自动完成此复制。

---

## 十、文件操作强制规则

### 创建新文件（三步法）
1. Write → 仅写文件头（#pragma once + 类骨架）≤ 30 行
2. Edit（循环）→ 逐块填充方法实现，每次 ≤ 50 行
3. Read → 每 3 次 Edit 后必须验证内容

### 修改已有文件
1. Read → 先读取目标区域上下文
2. Edit → 精确替换单处，≤ 50 行
3. Read → 验证后再继续

### 禁止行为
- 禁止一次性将整个类写入 Write
- 禁止单次 Edit 超过 50 行
- 禁止未读先改

---

## 十一、编码约定（C++17）

- `#pragma once`（禁止 `#ifndef` 风格）
- 禁止裸 new/delete（用 std::unique_ptr / std::shared_ptr）
- 禁止 using namespace std 在头文件中
- 析构函数加 noexcept
- 成员变量尾随下划线（如 controller_）
- 类名 PascalCase，函数名 camelCase
- 禁止魔法数字，枚举或 constexpr 常量替代
- 禁止在 UI 主线程执行耗时 > 5ms 的操作

---

## 十二、交付清单

完成后必须交付以下文件/改动：

| 序号 | 文件 | 操作类型 |
|---|---|---|
| 1 | twoProjector.ui | 修改（capWidget 新增三个控件组） |
| 2 | src/common/ProjectorManager.h | 新建 |
| 3 | src/common/ProjectorManager.cpp | 新建 |
| 4 | twoProjector.h（主窗口头文件） | 修改（添加 projMgr_ 成员和 setupProjectorManager 声明） |
| 5 | twoProjector.cpp（主窗口实现） | 修改（setupProjectorManager + 所有信号槽绑定） |
| 6 | build.bat 或 .vcxproj | 修改（追加 xcopy cyusbserial.dll 步骤） |

---

## 十三、执行顺序

1. 读取 f:\project\graduateSoftware\twoProjector.ui — 确认 capWidget 现有控件树
2. 读取 twoProjector.h 和 twoProjector.cpp — 确认主窗口现有结构
3. 读取 f:\project\graduateSoftware\Project2\Project2\JMTIController.h
4. 读取 f:\project\graduateSoftware\Project2\Project2\JMTIController.cpp
5. 按三步法修改 twoProjector.ui（添加新分组控件）
6. 按三步法新建 ProjectorManager.h 骨架
7. 按三步法填充 ProjectorManager.cpp 实现
8. 修改 twoProjector.h — 添加成员声明
9. 修改 twoProjector.cpp — 实现 setupProjectorManager 和所有信号槽
10. 修改 build.bat — 追加 DLL 复制步骤
11. Read 验证所有修改文件完整性

---

## 十四、禁止事项（最高优先级）

- 修改 F:\project\Envlib 内任何文件（P02）
- 手动编辑 ui_twoProjector.h（P05）
- 在 UI 主线程调用任何 JMTIController 方法（P03：会阻塞 I2C 总线等待）
- 修改 resWidget / bgaWidget 的任何已有控件
- 修改 Project2 源文件（只读参考）

---

这是一个投影仪控制集成任务，请严格按照上述规范逐步完成。
```
