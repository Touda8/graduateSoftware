#include "MainWindow.h"
#include "ui_twoProjector.h"
#include "visualization/VtkWidget.h"
#include "common/Config.h"
#include "common/Logger.h"
#include "common/Exceptions.h"
#include "common/ProjectorManager.h"
#include <QFileDialog>
#include <QDateTime>
#include <QCoreApplication>
#include <QTimer>
#include <QDir>
#include <QVBoxLayout>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>

namespace {
// Windows 上 cv::imread 使用 ANSI 代码页，无法处理 UTF-8 中文路径。
// 通过 std::filesystem::path (wchar_t) + cv::imdecode 绕过此限制。
cv::Mat imreadSafe(const std::string& utf8Path, int flags) {
    std::ifstream ifs(std::filesystem::u8path(utf8Path), std::ios::binary);
    if (!ifs) return {};
    std::vector<uchar> buf(std::istreambuf_iterator<char>(ifs), {});
    return cv::imdecode(buf, flags);
}

// 将 CV_64F 相位图转为伪彩色 QPixmap
QPixmap phaseToPixmap(const cv::Mat& phaseMat) {
    if (phaseMat.empty()) return {};
    cv::Mat norm;
    cv::normalize(phaseMat, norm, 0, 255, cv::NORM_MINMAX);
    norm.convertTo(norm, CV_8U);
    cv::Mat color;
    cv::applyColorMap(norm, color, cv::COLORMAP_JET);
    cv::cvtColor(color, color, cv::COLOR_BGR2RGB);
    QImage img(color.data, color.cols, color.rows,
               static_cast<int>(color.step), QImage::Format_RGB888);
    return QPixmap::fromImage(img.copy());
}
} // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::twoProjectorClass)
{
    ui->setupUi(this);
    initVtkWidget();
    setupConnections();
    setupProjectorManager();
    tp::Config::instance().load(this);
    setState(State::IDLE);
    tp::Logger::instance().info("MainWindow constructor completed successfully");
}

MainWindow::~MainWindow() noexcept {
    tp::Config::instance().save(this);
    delete projMgr_;
    delete ui;
}

// setupConnections / initVtkWidget / slot implementations below

void MainWindow::initVtkWidget() {
    // resCloudLabel 位于 QVBoxLayout (resCloudLayout) 的第 2 项，stretch=20
    // 需要从布局中替换而非仅设置 geometry，否则 VtkWidget 不受布局管理
    auto* layout = qobject_cast<QVBoxLayout*>(ui->resCloudLabel->parentWidget()->layout());
    if (layout) {
        int idx = layout->indexOf(ui->resCloudLabel);
        delete ui->resCloudLabel;          // Qt 自动从 layout 移除
        ui->resCloudLabel = nullptr;
        vtkWidget_ = new tp::VtkWidget();
        layout->insertWidget(idx, vtkWidget_, 20);  // 保持 stretch=20
    } else {
        // fallback：无布局时直接替换
        auto* vtkWgt = new tp::VtkWidget(ui->resCloudLabel->parentWidget());
        vtkWgt->setGeometry(ui->resCloudLabel->geometry());
        delete ui->resCloudLabel;
        ui->resCloudLabel = nullptr;
        vtkWidget_ = vtkWgt;
    }
}

void MainWindow::setupConnections() {
    // ---- resWidget: 文件设置按钮 ----
    connect(ui->resBtnCalPro1Browse, &QPushButton::clicked,
            this, &MainWindow::onResBtnCalPro1BrowseClicked);
    connect(ui->resBtnCalPro2Browse, &QPushButton::clicked,
            this, &MainWindow::onResBtnCalPro2BrowseClicked);
    connect(ui->resBtnScanBrowse, &QPushButton::clicked,
            this, &MainWindow::onResBtnScanBrowseClicked);
    connect(ui->resBtnEpiBrowse, &QPushButton::clicked,
            this, &MainWindow::onResBtnEpiBrowseClicked);
    connect(ui->resBtnSavePathBrowse, &QPushButton::clicked,
            this, &MainWindow::onResBtnSavePathBrowseClicked);
    connect(ui->resBtnTogglePanel, &QPushButton::clicked,
            this, &MainWindow::onResBtnTogglePanelClicked);

    // ---- resWidget: 重建控制按钮 ----
    connect(ui->resBtnLoadCalib, &QPushButton::clicked,
            this, &MainWindow::onResBtnLoadCalibClicked);
    connect(ui->resBtnStartRebuild, &QPushButton::clicked,
            this, &MainWindow::onResBtnStartRebuildClicked);
    connect(ui->resBtnSaveCloud, &QPushButton::clicked,
            this, &MainWindow::onResBtnSaveCloudClicked);

    // ---- resWidget: 重建参数控件 ----
    connect(ui->resComboDecodeType, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onResComboDecodeTypeChanged);
    connect(ui->resCheckOrdered, &QCheckBox::stateChanged,
            this, &MainWindow::onResCheckOrderedChanged);
    connect(ui->resSpinFreq1, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onResSpinFreq1Changed);
    connect(ui->resSpinFreq2, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onResSpinFreq2Changed);
    connect(ui->resSpinFreq3, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onResSpinFreq3Changed);
    connect(ui->resSpinShift1, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onResSpinShift1Changed);
    connect(ui->resSpinShift2, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onResSpinShift2Changed);
    connect(ui->resSpinShift3, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onResSpinShift3Changed);
    connect(ui->resDblSpinModThresh, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinModThreshChanged);
    connect(ui->resCheckDepthRange, &QCheckBox::stateChanged,
            this, &MainWindow::onResCheckDepthRangeChanged);
    connect(ui->resDblSpinEpiThresh, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinEpiThreshChanged);
    connect(ui->resDblSpinCentroidThresh, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinCentroidThreshChanged);
    connect(ui->resDblSpinTmax, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinTmaxChanged);
    connect(ui->resDblSpinNumStable, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinNumStableChanged);
    connect(ui->resDblSpinFSNRHigh, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinFSNRHighChanged);
    connect(ui->resDblSpinFSNRLow, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinFSNRLowChanged);
    connect(ui->resDblSpinModHigh, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinModHighChanged);
    connect(ui->resDblSpinModLow, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onResDblSpinModLowChanged);
    connect(ui->resComboSaveMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onResComboSaveModeChanged);
    connect(ui->resSpinRepeatCount, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onResSpinRepeatCountChanged);
    connect(ui->resCheckGrid, &QCheckBox::stateChanged,
            this, &MainWindow::onResCheckGridChanged);
    connect(ui->resCheckWireframe, &QCheckBox::stateChanged,
            this, &MainWindow::onResCheckWireframeChanged);
    connect(ui->resCheckInvert, &QCheckBox::stateChanged,
            this, &MainWindow::onResCheckInvertChanged);

    // ---- 点云处理按钮 ----
    connect(ui->pcBtnImportCloud, &QPushButton::clicked,
            this, &MainWindow::onPcBtnImportCloudClicked);
    connect(ui->pcBtnCropROI, &QPushButton::clicked,
            this, &MainWindow::onPcBtnCropROIClicked);
    connect(ui->pcBtnCropPlane, &QPushButton::clicked,
            this, &MainWindow::onPcBtnCropPlaneClicked);
    connect(ui->pcBtnSubsample, &QPushButton::clicked,
            this, &MainWindow::onPcBtnSubsampleClicked);
    connect(ui->pcBtnMerge, &QPushButton::clicked,
            this, &MainWindow::onPcBtnMergeClicked);
    connect(ui->pcBtnDuplicate, &QPushButton::clicked,
            this, &MainWindow::onPcBtnDuplicateClicked);
    connect(ui->pcBtnSOR, &QPushButton::clicked,
            this, &MainWindow::onPcBtnSORClicked);
    connect(ui->pcBtnROR, &QPushButton::clicked,
            this, &MainWindow::onPcBtnRORClicked);
    connect(ui->pcBtnPassThrough, &QPushButton::clicked,
            this, &MainWindow::onPcBtnPassThroughClicked);
    connect(ui->pcBtnGaussian, &QPushButton::clicked,
            this, &MainWindow::onPcBtnGaussianClicked);
    connect(ui->pcBtnFitPlane, &QPushButton::clicked,
            this, &MainWindow::onPcBtnFitPlaneClicked);
    connect(ui->pcBtnNormal, &QPushButton::clicked,
            this, &MainWindow::onPcBtnNormalClicked);
    connect(ui->pcBtnCurvature, &QPushButton::clicked,
            this, &MainWindow::onPcBtnCurvatureClicked);
    connect(ui->pcBtnRoughness, &QPushButton::clicked,
            this, &MainWindow::onPcBtnRoughnessClicked);
    connect(ui->pcBtnICP, &QPushButton::clicked,
            this, &MainWindow::onPcBtnICPClicked);
    connect(ui->pcBtnDistance, &QPushButton::clicked,
            this, &MainWindow::onPcBtnDistanceClicked);
    connect(ui->pcBtnPCAAlign, &QPushButton::clicked,
            this, &MainWindow::onPcBtnPCAAlignClicked);
    connect(ui->pcBtnCenter, &QPushButton::clicked,
            this, &MainWindow::onPcBtnCenterClicked);
    connect(ui->pcBtnLevelPlane, &QPushButton::clicked,
            this, &MainWindow::onPcBtnLevelPlaneClicked);
    connect(ui->pcBtnApplyTransform, &QPushButton::clicked,
            this, &MainWindow::onPcBtnApplyTransformClicked);
    connect(ui->pcBtnViewFront, &QPushButton::clicked,
            this, &MainWindow::onPcBtnViewFrontClicked);
    connect(ui->pcBtnViewBack, &QPushButton::clicked,
            this, &MainWindow::onPcBtnViewBackClicked);
    connect(ui->pcBtnViewLeft, &QPushButton::clicked,
            this, &MainWindow::onPcBtnViewLeftClicked);
    connect(ui->pcBtnViewRight, &QPushButton::clicked,
            this, &MainWindow::onPcBtnViewRightClicked);
    connect(ui->pcBtnViewTop, &QPushButton::clicked,
            this, &MainWindow::onPcBtnViewTopClicked);
    connect(ui->pcBtnViewBottom, &QPushButton::clicked,
            this, &MainWindow::onPcBtnViewBottomClicked);
    connect(ui->pcBtnViewIsometric, &QPushButton::clicked,
            this, &MainWindow::onPcBtnViewIsometricClicked);
    connect(ui->pcBtnViewReset, &QPushButton::clicked,
            this, &MainWindow::onPcBtnViewResetClicked);
    connect(ui->pcBtnExportPLY, &QPushButton::clicked,
            this, &MainWindow::onPcBtnExportPLYClicked);
    connect(ui->pcBtnExportPCD, &QPushButton::clicked,
            this, &MainWindow::onPcBtnExportPCDClicked);
    connect(ui->pcBtnExportCSV, &QPushButton::clicked,
            this, &MainWindow::onPcBtnExportCSVClicked);
    connect(ui->pcBtnScreenshot, &QPushButton::clicked,
            this, &MainWindow::onPcBtnScreenshotClicked);

    // ---- 点云参数控件 ----
    connect(ui->pcSpinVoxelSize, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinVoxelSizeChanged);
    connect(ui->pcSpinSORK, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onPcSpinSORKChanged);
    connect(ui->pcSpinSORStd, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinSORStdChanged);
    connect(ui->pcSpinRORRadius, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinRORRadiusChanged);
    connect(ui->pcSpinRORMin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onPcSpinRORMinChanged);
    connect(ui->pcComboPassAxis, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onPcComboPassAxisChanged);
    connect(ui->pcComboFitMethod, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onPcComboFitMethodChanged);
    connect(ui->pcSpinNormalR, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinNormalRChanged);
    connect(ui->pcSpinRotX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinRotXChanged);
    connect(ui->pcSpinRotY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinRotYChanged);
    connect(ui->pcSpinRotZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinRotZChanged);
    connect(ui->pcSpinTransX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinTransXChanged);
    connect(ui->pcSpinTransY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinTransYChanged);
    connect(ui->pcSpinTransZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onPcSpinTransZChanged);

    // ---- bgaWidget ----
    connect(ui->bgaSpinCount, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onBgaSpinCountChanged);
    connect(ui->bgaBtnStart, &QPushButton::clicked,
            this, &MainWindow::onBgaBtnStartClicked);
    connect(ui->bgaBtnPlot, &QPushButton::clicked,
            this, &MainWindow::onBgaBtnPlotClicked);
    connect(ui->bgaBtnBallShow, &QPushButton::clicked,
            this, &MainWindow::onBgaBtnBallShowClicked);
    connect(ui->bgaBtnImport, &QPushButton::clicked,
            this, &MainWindow::onBgaBtnImportClicked);
    connect(ui->bgaBtnSave, &QPushButton::clicked,
            this, &MainWindow::onBgaBtnSaveClicked);

    // ---- qfpWidget ----
    connect(ui->qfpSpinCount, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::onQfpSpinCountChanged);
    connect(ui->qfpBtnStart, &QPushButton::clicked,
            this, &MainWindow::onQfpBtnStartClicked);
    connect(ui->qfpBtnPlot, &QPushButton::clicked,
            this, &MainWindow::onQfpBtnPlotClicked);
    connect(ui->qfpBtnPinShow, &QPushButton::clicked,
            this, &MainWindow::onQfpBtnPinShowClicked);
    connect(ui->qfpBtnImport, &QPushButton::clicked,
            this, &MainWindow::onQfpBtnImportClicked);
    connect(ui->qfpBtnSave, &QPushButton::clicked,
            this, &MainWindow::onQfpBtnSaveClicked);

    // ---- 内部日志/进度信号 (QueuedConnection for thread safety) ----
    connect(this, &MainWindow::logMessage,
            this, &MainWindow::appendLog, Qt::QueuedConnection);
    connect(this, &MainWindow::progressUpdated,
            ui->resProgressBar, &QProgressBar::setValue, Qt::QueuedConnection);
}

// ============================================================
// 状态管理与日志
// ============================================================

void MainWindow::setState(State s) {
    state_ = s;
    const bool idle  = (s == State::IDLE);
    const bool ready = (s == State::READY);
    const bool done  = (s == State::DONE);
    const bool busy  = (s == State::RECONSTRUCTING || s == State::CALIB_LOADING);

    ui->resBtnLoadCalib->setEnabled(!busy);
    ui->resBtnStartRebuild->setEnabled(ready);
    ui->resBtnSaveCloud->setEnabled(done);
}

void MainWindow::appendLog(const QString& msg) {
    ui->resLogText->append(
        QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg);
}

void MainWindow::appendCapLog(const QString& msg) {
    ui->capLogText->append(
        QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg);
}

void MainWindow::updateProStatusLabel(int projIdx, bool connected) {
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

    // Update capTreeWidget node: Pro1→topLevelItem(1)/child(0), Pro2→child(1)
    QTreeWidgetItem* projParent = ui->capTreeWidget->topLevelItem(1);
    if (projParent && projIdx < projParent->childCount()) {
        projParent->child(projIdx)->setText(1, connected ? "已连接" : "未连接");
    }

    // Sync all-connect / all-disconnect button states
    bool anyConn = projMgr_->connectedCount() > 0;
    ui->capBtnAllDisconnect->setEnabled(anyConn);
}

// ============================================================
// capWidget — 投影仪管理初始化
// ============================================================

void MainWindow::setupProjectorManager() {
    projMgr_ = new ProjectorManager();

    // ---- 连接管理按钮（带禁用防重复点击） ----
    connect(ui->capBtnProjScan, &QPushButton::clicked,
            this, [this] {
                ui->capBtnProjScan->setEnabled(false);
                appendCapLog(QStringLiteral("开始扫描投影仪设备..."));
                projMgr_->scanDevices();
            });
    connect(ui->capBtnPro1Connect, &QPushButton::clicked,
            this, [this] {
                ui->capBtnPro1Connect->setEnabled(false);
                appendCapLog(QStringLiteral("正在连接 Pro1..."));
                projMgr_->connectDevice(ProjectorManager::PRO1_IDX);
            });
    connect(ui->capBtnPro1Disconnect, &QPushButton::clicked,
            this, [this] {
                appendCapLog(QStringLiteral("正在断开 Pro1..."));
                projMgr_->disconnectDevice(ProjectorManager::PRO1_IDX);
            });
    connect(ui->capBtnPro2Connect, &QPushButton::clicked,
            this, [this] {
                ui->capBtnPro2Connect->setEnabled(false);
                appendCapLog(QStringLiteral("正在连接 Pro2..."));
                projMgr_->connectDevice(ProjectorManager::PRO2_IDX);
            });
    connect(ui->capBtnPro2Disconnect, &QPushButton::clicked,
            this, [this] {
                appendCapLog(QStringLiteral("正在断开 Pro2..."));
                projMgr_->disconnectDevice(ProjectorManager::PRO2_IDX);
            });
    connect(ui->capBtnAllConnect, &QPushButton::clicked,
            this, [this] {
                ui->capBtnAllConnect->setEnabled(false);
                ui->capBtnPro1Connect->setEnabled(false);
                ui->capBtnPro2Connect->setEnabled(false);
                appendCapLog(QStringLiteral("开始连接全部投影仪..."));
                projMgr_->connectAll();
            });
    connect(ui->capBtnAllDisconnect, &QPushButton::clicked,
            this, [this] {
                appendCapLog(QStringLiteral("开始断开全部投影仪..."));
                projMgr_->disconnectAll();
            });

    // ---- 状态反馈信号 ----
    connect(projMgr_, &ProjectorManager::connectionStateChanged,
            this, &MainWindow::updateProStatusLabel,
            Qt::QueuedConnection);
    connect(projMgr_, &ProjectorManager::statusMessage,
            this, &MainWindow::appendCapLog,
            Qt::QueuedConnection);
    connect(projMgr_, &ProjectorManager::errorOccurred,
            this, [this](int, QString err) { appendCapLog(QStringLiteral("错误: ") + err); },
            Qt::QueuedConnection);
    connect(projMgr_, &ProjectorManager::deviceListUpdated,
            this, [this](QStringList list) {
                QTreeWidgetItem* projParent = ui->capTreeWidget->topLevelItem(1);
                if (!projParent) return;
                for (int i = 0; i < projParent->childCount() && i < list.size(); ++i) {
                    projParent->child(i)->setToolTip(0, list[i]);
                }
            }, Qt::QueuedConnection);
    connect(projMgr_, &ProjectorManager::operationFinished,
            this, [this] {
                ui->capBtnProjScan->setEnabled(true);
                ui->capBtnAllConnect->setEnabled(true);
                // 连接/断开按钮状态由实际连接情况决定
                bool pro1Conn = projMgr_->isConnected(0);
                bool pro2Conn = projMgr_->isConnected(1);
                ui->capBtnPro1Connect->setEnabled(!pro1Conn);
                ui->capBtnPro1Disconnect->setEnabled(pro1Conn);
                ui->capBtnPro2Connect->setEnabled(!pro2Conn);
                ui->capBtnPro2Disconnect->setEnabled(pro2Conn);
                ui->capBtnAllDisconnect->setEnabled(pro1Conn || pro2Conn);
            }, Qt::QueuedConnection);

    // ---- 测试图案按钮 ----
    connect(ui->capBtnProjShowTest, &QPushButton::clicked, this, [this] {
        bool pro1 = ui->capCheckPro1Test->isChecked();
        bool pro2 = ui->capCheckPro2Test->isChecked();
        if (!pro1 && !pro2) {
            appendCapLog(QStringLiteral("错误: 请至少勾选一个投影仪"));
            return;
        }
        int pat = ui->capComboProjPattern->currentIndex();
        static constexpr int PAT_MAP[] = {0,0,8,3,5,6,7};
        static constexpr int COL_MAP[] = {7,0,7,7,7,7,7};
        int patCode = PAT_MAP[pat];
        int colCode = COL_MAP[pat];
        if (pro1) {
            if (!projMgr_->isConnected(0)) {
                appendCapLog(QStringLiteral("错误: Pro1 未连接，请先连接设备"));
            } else {
                projMgr_->showTestPattern(0, patCode, colCode);
            }
        }
        if (pro2) {
            if (!projMgr_->isConnected(1)) {
                appendCapLog(QStringLiteral("错误: Pro2 未连接，请先连接设备"));
            } else {
                projMgr_->showTestPattern(1, patCode, colCode);
            }
        }
    });
    connect(ui->capBtnProjStopTest, &QPushButton::clicked, this, [this] {
        bool pro1 = ui->capCheckPro1Test->isChecked();
        bool pro2 = ui->capCheckPro2Test->isChecked();
        if (!pro1 && !pro2) {
            appendCapLog(QStringLiteral("错误: 请至少勾选一个投影仪"));
            return;
        }
        if (pro1) {
            projMgr_->stopTestPattern(0);
        }
        if (pro2) {
            projMgr_->stopTestPattern(1);
        }
    });

    // ---- Pro1/Pro2 投射按钮（条纹投射） ----
    connect(ui->capBtnPro1Trig, &QPushButton::clicked, this, [this] {
        if (!projMgr_->isConnected(0)) {
            appendCapLog(QStringLiteral("错误: Pro1 未连接，请先扫描并连接设备"));
            return;
        }
        appendCapLog(QStringLiteral("Pro1 开始条纹投射..."));
        projMgr_->startPatternSequence(0, true);
    });
    connect(ui->capBtnPro2Trig, &QPushButton::clicked, this, [this] {
        if (!projMgr_->isConnected(1)) {
            appendCapLog(QStringLiteral("错误: Pro2 未连接，请先扫描并连接设备"));
            return;
        }
        appendCapLog(QStringLiteral("Pro2 开始条纹投射..."));
        projMgr_->startPatternSequence(1, true);
    });

    // ---- 连续采集按钮（双投影仪循环投射切换） ----
    connect(ui->capBtnContinuous, &QPushButton::clicked, this, [this] {
        if (!continuousRunning_) {
            continuousRunning_ = true;
            ui->capBtnContinuous->setText(QStringLiteral("停止采集"));
            appendCapLog(QStringLiteral("开始连续投射 (Pro1 + Pro2)"));
            projMgr_->triggerAll(false);
        } else {
            continuousRunning_ = false;
            ui->capBtnContinuous->setText(QStringLiteral("连续采集"));
            appendCapLog(QStringLiteral("停止连续投射"));
            projMgr_->stopPatternSequence(0);
            projMgr_->stopPatternSequence(1);
        }
    });

    // ---- 文件浏览按钮 ----
    connect(ui->capBtnBrowse, &QPushButton::clicked, this, [this] {
        auto dir = QFileDialog::getExistingDirectory(this,
            QStringLiteral("选择采集图像文件夹"));
        if (!dir.isEmpty()) {
            ui->capEditFolder->setText(dir);
            appendCapLog(QStringLiteral("采集文件夹设置为: ") + dir);
        }
    });
}

// ============================================================
// resWidget — 文件设置按钮
// ============================================================

void MainWindow::onResBtnCalPro1BrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择标定数据文件夹 (包含 X.raw, allK_i1.raw 等)"));
    if (!dir.isEmpty()) ui->resEditCalPro1->setText(dir);
}

void MainWindow::onResBtnCalPro2BrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择标定数据文件夹 (备用)"));
    if (!dir.isEmpty()) ui->resEditCalPro2->setText(dir);
}

void MainWindow::onResBtnScanBrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择扫描图像文件夹"));
    if (!dir.isEmpty()) ui->resEditScanFolder->setText(dir);
}

void MainWindow::onResBtnEpiBrowseClicked() {
    auto path = QFileDialog::getOpenFileName(this,
        QStringLiteral("选择极线参数文件"), {}, "MAT Files (*.mat)");
    if (!path.isEmpty()) ui->resEditEpiFile->setText(path);
}

void MainWindow::onResBtnSavePathBrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择保存路径"));
    if (!dir.isEmpty()) ui->resEditSavePath->setText(dir);
}

void MainWindow::onResBtnTogglePanelClicked() {
    bool vis = ui->resLeftWidget->isVisible();
    ui->resLeftWidget->setVisible(!vis);
    ui->resBtnTogglePanel->setText(vis
        ? QStringLiteral("▶ 展开参数面板")
        : QStringLiteral("◀ 收起参数面板"));
}

// ============================================================
// resWidget — 重建控制按钮
// ============================================================

void MainWindow::onResBtnLoadCalibClicked() {
    appendLog(QStringLiteral("加载标定文件..."));
    setState(State::CALIB_LOADING);
    emit progressUpdated(0);

    // 解析标定目录：优先使用用户填写路径，否则按多级回退查找
    QString calibDir = ui->resEditCalPro1->text().trimmed();
    auto resolveDir = [](const QString& input, const QString& defaultRel) -> QString {
        // 1. 用户指定路径存在则直接使用
        if (!input.isEmpty() && QDir(input).exists())
            return QDir(input).absolutePath();
        // 2. 相对于工作目录
        if (QDir(QDir::currentPath()).exists(defaultRel))
            return QDir(QDir::currentPath()).absoluteFilePath(defaultRel);
        // 3. 相对于 exe 所在目录的两级上层 (x64/Release → 项目根)
        QDir exeDir(QCoreApplication::applicationDirPath());
        if (exeDir.cdUp() && exeDir.cdUp() && exeDir.exists(defaultRel))
            return exeDir.absoluteFilePath(defaultRel);
        return input;  // 返回原始值，让后续 loadDual 抛出可读错误
    };
    calibDir = resolveDir(calibDir, QStringLiteral("data/calibrationResult/raw"));
    appendLog(QStringLiteral("标定路径: ") + calibDir);

    std::thread([this, dir = calibDir.toStdString()]() {
        try {
            emit progressUpdated(20);
            tp::CalibLoader loader;
            auto calib = loader.loadDual(dir);
            emit progressUpdated(80);
            if (!calib.isValid) {
                emit logMessage(QStringLiteral("标定数据无效"));
                QMetaObject::invokeMethod(this, [this]() {
                    setState(State::IDLE);
                }, Qt::QueuedConnection);
                return;
            }
            dualCalib_ = calib;
            emit progressUpdated(100);
            emit logMessage(QStringLiteral("标定加载成功: X=%1×%2, allK_i1=%3×%4")
                .arg(calib.X.rows).arg(calib.X.cols)
                .arg(calib.allK_i1.rows).arg(calib.allK_i1.cols));
            QMetaObject::invokeMethod(this, [this]() {
                setState(State::READY);
            }, Qt::QueuedConnection);
        } catch (const std::exception& e) {
            emit logMessage(QString("标定加载失败: ") + e.what());
            QMetaObject::invokeMethod(this, [this]() {
                setState(State::IDLE);
            }, Qt::QueuedConnection);
        }
    }).detach();
}

void MainWindow::onResBtnStartRebuildClicked() {
    if (state_ != State::READY) {
        appendLog(QStringLiteral("请先加载标定文件")); return;
    }
    appendLog(QStringLiteral("开始重建..."));
    setState(State::RECONSTRUCTING);
    emit progressUpdated(0);

    // 解析扫描目录：同 calibDir 逻辑
    QString scanDir = ui->resEditScanFolder->text().trimmed();
    auto resolveScanDir = [](const QString& input, const QString& defaultRel) -> QString {
        if (!input.isEmpty() && QDir(input).exists())
            return QDir(input).absolutePath();
        if (QDir(QDir::currentPath()).exists(defaultRel))
            return QDir(QDir::currentPath()).absoluteFilePath(defaultRel);
        QDir exeDir(QCoreApplication::applicationDirPath());
        if (exeDir.cdUp() && exeDir.cdUp() && exeDir.exists(defaultRel))
            return exeDir.absoluteFilePath(defaultRel);
        return input;
    };
    scanDir = resolveScanDir(scanDir, QStringLiteral("data/pic/BGA\u9759\u6001"));
    appendLog(QStringLiteral("\u626b\u63cf\u8def\u5f84: ") + scanDir);

    auto params = reconParams_;
    auto calib = dualCalib_;

    std::thread([this, dir = scanDir.toStdString(), params, calib]() {
        try {
            emit progressUpdated(5);
            emit logMessage(QStringLiteral("读取图像..."));

            auto loadImgs = [](const std::string& folder, int from, int to) {
                std::vector<cv::Mat> imgs;
                for (int i = from; i <= to; ++i) {
                    auto path = folder + "/" + std::to_string(i) + ".bmp";
                    cv::Mat img = imreadSafe(path, cv::IMREAD_GRAYSCALE);
                    if (img.empty())
                        throw tp::ImageLoadException("Cannot load: " + path);
                    cv::Mat d; img.convertTo(d, CV_64F, 1.0 / 255.0);
                    imgs.push_back(d);
                }
                return imgs;
            };

            auto imgs1 = loadImgs(dir + "/1", 1, 24);
            emit progressUpdated(15);
            auto imgs2 = loadImgs(dir + "/2", 1, 24);
            emit progressUpdated(25);

            cv::Mat colorImg = imreadSafe(dir + "/1/49.bmp", cv::IMREAD_GRAYSCALE);
            if (colorImg.empty())
                colorImg = imreadSafe(dir + "/2/49.bmp", cv::IMREAD_GRAYSCALE);
            if (colorImg.empty() && !imgs1.empty()) {
                colorImg = cv::Mat(imgs1[0].rows, imgs1[0].cols, CV_8U, cv::Scalar(128));
                emit logMessage(QStringLiteral("警告：第49帧参考图不存在，使用纯灰替代"));
            }

            emit logMessage(QStringLiteral("执行重建流程..."));
            emit progressUpdated(30);

            tp::ReconstructionPipeline pipeline;
            auto result = pipeline.runDual(imgs1, imgs2, colorImg, calib, params);
            emit progressUpdated(90);

            if (!result.success) {
                emit logMessage(QString("重建失败: ") +
                    QString::fromStdString(result.errorMsg));
                QMetaObject::invokeMethod(this, [this]() {
                    setState(State::READY);
                }, Qt::QueuedConnection);
                return;
            }

            auto colorCloud = result.cloud;
            emit progressUpdated(100);
            emit logMessage(QStringLiteral("重建完成: %1 点").arg(colorCloud->size()));

            // Worker 线程中只生成 QImage（线程安全），QPixmap 转换在 Main 线程
            auto makeQImage = [](const cv::Mat& phaseMat) -> QImage {
                if (phaseMat.empty()) return {};
                cv::Mat norm;
                cv::normalize(phaseMat, norm, 0, 255, cv::NORM_MINMAX);
                norm.convertTo(norm, CV_8U);
                cv::Mat color;
                cv::applyColorMap(norm, color, cv::COLORMAP_JET);
                cv::cvtColor(color, color, cv::COLOR_BGR2RGB);
                cv::Mat cloned = color.clone();
                return QImage(cloned.data, cloned.cols, cloned.rows,
                              static_cast<int>(cloned.step),
                              QImage::Format_RGB888).copy();
            };
            QImage img1 = makeQImage(result.absPhase1);
            QImage img2 = makeQImage(result.absPhase2);

            QMetaObject::invokeMethod(this, [this, colorCloud, img1, img2]() {
                // 所有 Qt GUI 操作在 Main 线程
                currentCloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                for (const auto& pt : colorCloud->points)
                    currentCloud_->push_back({pt.x, pt.y, pt.z});
                vtkWidget_->updateCloud(colorCloud);
                if (!img1.isNull())
                    ui->resPhaseLabel1->setPixmap(QPixmap::fromImage(img1));
                if (!img2.isNull())
                    ui->resPhaseLabel2->setPixmap(QPixmap::fromImage(img2));
                setState(State::DONE);
            }, Qt::QueuedConnection);

        } catch (const std::exception& e) {
            emit logMessage(QString("重建错误: ") + e.what());
            QMetaObject::invokeMethod(this, [this]() {
                setState(State::READY);
            }, Qt::QueuedConnection);
        }
    }).detach();
}

void MainWindow::onResBtnSaveCloudClicked() {
    if (!currentCloud_ || currentCloud_->empty()) {
        appendLog(QStringLiteral("没有点云可保存")); return;
    }
    auto path = QFileDialog::getSaveFileName(this, QStringLiteral("保存点云"), {},
        "PLY(*.ply);;PCD(*.pcd);;CSV(*.csv)");
    if (path.isEmpty()) return;
    try {
        auto ext = path.section('.', -1).toLower();
        if (ext == "ply") tp::PointCloudIO::savePLY(currentCloud_, path.toStdString());
        else if (ext == "pcd") tp::PointCloudIO::savePCD(currentCloud_, path.toStdString());
        else tp::PointCloudIO::saveCSV(currentCloud_, path.toStdString());
        appendLog(QStringLiteral("已保存: ") + path);
    } catch (const std::exception& e) {
        appendLog(QString("保存失败: ") + e.what());
    }
}

// ============================================================
// resWidget — 重建参数变更
// ============================================================

void MainWindow::onResComboDecodeTypeChanged(int index) {
    reconParams_.decodeType = (index == 0) ? "multi_freq" : "temporal";
}

void MainWindow::onResCheckOrderedChanged(int state) {
    reconParams_.orderedCloud = (state == Qt::Checked);
}

void MainWindow::onResSpinFreq1Changed(int val) { reconParams_.freq1 = val; }
void MainWindow::onResSpinFreq2Changed(int val) { reconParams_.freq2 = val; }
void MainWindow::onResSpinFreq3Changed(int val) { reconParams_.freq3 = val; }
void MainWindow::onResSpinShift1Changed(int val) { reconParams_.shift1 = val; }
void MainWindow::onResSpinShift2Changed(int val) { reconParams_.shift2 = val; }
void MainWindow::onResSpinShift3Changed(int val) { reconParams_.shift3 = val; }

void MainWindow::onResDblSpinModThreshChanged(double val) {
    reconParams_.modThresh = val;
}

void MainWindow::onResCheckDepthRangeChanged(int state) {
    reconParams_.depthRangeEnabled = (state == Qt::Checked);
}

void MainWindow::onResDblSpinEpiThreshChanged(double val) {
    reconParams_.epiThresh = val;
}
void MainWindow::onResDblSpinCentroidThreshChanged(double val) {
    reconParams_.centroidThresh = val;
}
void MainWindow::onResDblSpinTmaxChanged(double val) {
    reconParams_.tMax = val;
}
void MainWindow::onResDblSpinNumStableChanged(double val) {
    reconParams_.numStable = val;
}
void MainWindow::onResDblSpinFSNRHighChanged(double val) {
    reconParams_.fsnrHigh = val;
}
void MainWindow::onResDblSpinFSNRLowChanged(double val) {
    reconParams_.fsnrLow = val;
}
void MainWindow::onResDblSpinModHighChanged(double val) {
    reconParams_.modHigh = val;
}
void MainWindow::onResDblSpinModLowChanged(double val) {
    reconParams_.modLow = val;
}

void MainWindow::onResComboSaveModeChanged(int index) {
    ui->resSpinRepeatCount->setEnabled(index == 1);
}

void MainWindow::onResSpinRepeatCountChanged(int val) {
    reconParams_.repeatCount = val;
}

void MainWindow::onResCheckGridChanged(int state) {
    if (vtkWidget_) vtkWidget_->setGridVisible(state == Qt::Checked);
}
void MainWindow::onResCheckWireframeChanged(int state) {
    if (vtkWidget_) vtkWidget_->setWireframe(state == Qt::Checked);
}
void MainWindow::onResCheckInvertChanged(int state) {
    // 高度反转是可视化层高级功能，暂留
}

// ============================================================
// 点云处理按钮
// ============================================================

void MainWindow::onPcBtnImportCloudClicked() {
    auto path = QFileDialog::getOpenFileName(this,
        QStringLiteral("导入点云"), {},
        "Point Cloud (*.ply *.pcd)");
    if (path.isEmpty()) return;
    try {
        currentCloud_ = tp::PointCloudIO::load(path.toStdString());
        vtkWidget_->updateCloud(currentCloud_);
        appendLog(QStringLiteral("导入点云: ") + path +
            QStringLiteral(" (%1 点)").arg(currentCloud_->size()));
    } catch (const std::exception& e) {
        appendLog(QString("导入失败: ") + e.what());
    }
}

void MainWindow::onPcBtnCropROIClicked() {
    appendLog(QStringLiteral("ROI 裁剪功能开发中"));
}
void MainWindow::onPcBtnCropPlaneClicked() {
    appendLog(QStringLiteral("平面裁剪功能开发中"));
}
void MainWindow::onPcBtnSubsampleClicked() {
    if (!currentCloud_ || currentCloud_->empty()) {
        appendLog(QStringLiteral("请先导入或重建点云")); return;
    }
    float leafSize = static_cast<float>(ui->pcSpinVoxelSize->value());
    auto cloud = currentCloud_;
    std::thread([this, cloud, leafSize]() {
        try {
            tp::PointCloudFilter f;
            auto filtered = f.voxelGrid(cloud, leafSize);
            QMetaObject::invokeMethod(this, [this, filtered]() {
                currentCloud_ = filtered;
                vtkWidget_->updateCloud(filtered);
                appendLog(QStringLiteral("体素降采样完成: %1 点").arg(filtered->size()));
            }, Qt::QueuedConnection);
        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString(e.what())]() {
                appendLog("降采样错误: " + msg);
            }, Qt::QueuedConnection);
        }
    }).detach();
}
void MainWindow::onPcBtnMergeClicked() {
    appendLog(QStringLiteral("合并点云功能开发中"));
}
void MainWindow::onPcBtnDuplicateClicked() {
    appendLog(QStringLiteral("去除重复点功能开发中"));
}
void MainWindow::onPcBtnSORClicked() {
    if (!currentCloud_ || currentCloud_->empty()) {
        appendLog(QStringLiteral("请先导入或重建点云")); return;
    }
    int meanK = ui->pcSpinSORK->value();
    double stdDev = ui->pcSpinSORStd->value();
    auto cloud = currentCloud_;
    std::thread([this, cloud, meanK, stdDev]() {
        try {
            tp::PointCloudFilter f;
            auto filtered = f.SOR(cloud, meanK, stdDev);
            QMetaObject::invokeMethod(this, [this, filtered]() {
                currentCloud_ = filtered;
                vtkWidget_->updateCloud(filtered);
                appendLog(QStringLiteral("SOR完成: %1 点").arg(filtered->size()));
            }, Qt::QueuedConnection);
        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString(e.what())]() {
                appendLog("SOR错误: " + msg);
            }, Qt::QueuedConnection);
        }
    }).detach();
}
void MainWindow::onPcBtnRORClicked() {
    appendLog(QStringLiteral("ROR 滤波功能开发中"));
}
void MainWindow::onPcBtnPassThroughClicked() {
    if (!currentCloud_ || currentCloud_->empty()) {
        appendLog(QStringLiteral("请先导入或重建点云")); return;
    }
    int axisIdx = ui->pcComboPassAxis->currentIndex();
    std::string axis = (axisIdx == 0) ? "x" : (axisIdx == 1) ? "y" : "z";
    auto cloud = currentCloud_;
    std::thread([this, cloud, axis]() {
        try {
            tp::PointCloudFilter f;
            auto filtered = f.passThrough(cloud, axis, -999.0f, 999.0f);
            QMetaObject::invokeMethod(this, [this, filtered]() {
                currentCloud_ = filtered;
                vtkWidget_->updateCloud(filtered);
                appendLog(QStringLiteral("直通滤波完成: %1 点").arg(filtered->size()));
            }, Qt::QueuedConnection);
        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString(e.what())]() {
                appendLog("直通滤波错误: " + msg);
            }, Qt::QueuedConnection);
        }
    }).detach();
}
void MainWindow::onPcBtnGaussianClicked() {
    appendLog(QStringLiteral("高斯平滑功能开发中"));
}
void MainWindow::onPcBtnFitPlaneClicked() {
    appendLog(QStringLiteral("平面拟合功能开发中"));
}
void MainWindow::onPcBtnNormalClicked() {
    appendLog(QStringLiteral("法线估计功能开发中"));
}
void MainWindow::onPcBtnCurvatureClicked() {
    appendLog(QStringLiteral("曲率计算功能开发中"));
}
void MainWindow::onPcBtnRoughnessClicked() {
    appendLog(QStringLiteral("粗糙度计算功能开发中"));
}
void MainWindow::onPcBtnICPClicked() {
    appendLog(QStringLiteral("ICP 配准功能开发中"));
}
void MainWindow::onPcBtnDistanceClicked() {
    appendLog(QStringLiteral("距离映射功能开发中"));
}

void MainWindow::onPcBtnPCAAlignClicked() {
    appendLog(QStringLiteral("PCA 对齐功能开发中"));
}
void MainWindow::onPcBtnCenterClicked() {
    appendLog(QStringLiteral("居中原点功能开发中"));
}
void MainWindow::onPcBtnLevelPlaneClicked() {
    appendLog(QStringLiteral("基准平面校正功能开发中"));
}
void MainWindow::onPcBtnApplyTransformClicked() {
    appendLog(QStringLiteral("应用变换功能开发中"));
}
void MainWindow::onPcBtnViewFrontClicked() {
    if (vtkWidget_) vtkWidget_->setCameraPreset(tp::VtkWidget::ViewFront);
}
void MainWindow::onPcBtnViewBackClicked() {
    if (vtkWidget_) vtkWidget_->setCameraPreset(tp::VtkWidget::ViewBack);
}
void MainWindow::onPcBtnViewLeftClicked() {
    if (vtkWidget_) vtkWidget_->setCameraPreset(tp::VtkWidget::ViewLeft);
}
void MainWindow::onPcBtnViewRightClicked() {
    if (vtkWidget_) vtkWidget_->setCameraPreset(tp::VtkWidget::ViewRight);
}
void MainWindow::onPcBtnViewTopClicked() {
    if (vtkWidget_) vtkWidget_->setCameraPreset(tp::VtkWidget::ViewTop);
}
void MainWindow::onPcBtnViewBottomClicked() {
    if (vtkWidget_) vtkWidget_->setCameraPreset(tp::VtkWidget::ViewBottom);
}
void MainWindow::onPcBtnViewIsometricClicked() {
    if (vtkWidget_) vtkWidget_->setCameraPreset(tp::VtkWidget::ViewIsometric);
}
void MainWindow::onPcBtnViewResetClicked() {
    if (vtkWidget_) vtkWidget_->resetCamera();
}
void MainWindow::onPcBtnExportPLYClicked() {
    if (!currentCloud_ || currentCloud_->empty()) { appendLog(QStringLiteral("没有点云")); return; }
    auto path = QFileDialog::getSaveFileName(this, QStringLiteral("导出PLY"), {}, "PLY(*.ply)");
    if (path.isEmpty()) return;
    try {
        tp::PointCloudIO::savePLY(currentCloud_, path.toStdString());
        appendLog(QStringLiteral("已导出: ") + path);
    } catch (const std::exception& e) { appendLog(QString("导出错误: ") + e.what()); }
}
void MainWindow::onPcBtnExportPCDClicked() {
    if (!currentCloud_ || currentCloud_->empty()) { appendLog(QStringLiteral("没有点云")); return; }
    auto path = QFileDialog::getSaveFileName(this, QStringLiteral("导出PCD"), {}, "PCD(*.pcd)");
    if (path.isEmpty()) return;
    try {
        tp::PointCloudIO::savePCD(currentCloud_, path.toStdString());
        appendLog(QStringLiteral("已导出: ") + path);
    } catch (const std::exception& e) { appendLog(QString("导出错误: ") + e.what()); }
}
void MainWindow::onPcBtnExportCSVClicked() {
    if (!currentCloud_ || currentCloud_->empty()) { appendLog(QStringLiteral("没有点云")); return; }
    auto path = QFileDialog::getSaveFileName(this, QStringLiteral("导出CSV"), {}, "CSV(*.csv)");
    if (path.isEmpty()) return;
    try {
        tp::PointCloudIO::saveCSV(currentCloud_, path.toStdString());
        appendLog(QStringLiteral("已导出: ") + path);
    } catch (const std::exception& e) { appendLog(QString("导出错误: ") + e.what()); }
}
void MainWindow::onPcBtnScreenshotClicked() {
    auto path = QFileDialog::getSaveFileName(this, QStringLiteral("截图"), {}, "PNG(*.png)");
    if (path.isEmpty()) return;
    if (vtkWidget_) vtkWidget_->saveScreenshot(path.toStdString());
    appendLog(QStringLiteral("截图已保存: ") + path);
}

// ============================================================
// 点云参数控件
// ============================================================

void MainWindow::onPcSpinVoxelSizeChanged(double) {}
void MainWindow::onPcSpinSORKChanged(int) {}
void MainWindow::onPcSpinSORStdChanged(double) {}
void MainWindow::onPcSpinRORRadiusChanged(double) {}
void MainWindow::onPcSpinRORMinChanged(int) {}
void MainWindow::onPcComboPassAxisChanged(int) {}
void MainWindow::onPcComboFitMethodChanged(int) {}
void MainWindow::onPcSpinNormalRChanged(double) {}
void MainWindow::onPcSpinRotXChanged(double) {}
void MainWindow::onPcSpinRotYChanged(double) {}
void MainWindow::onPcSpinRotZChanged(double) {}
void MainWindow::onPcSpinTransXChanged(double) {}
void MainWindow::onPcSpinTransYChanged(double) {}
void MainWindow::onPcSpinTransZChanged(double) {}

// ============================================================
// bgaWidget
// ============================================================

void MainWindow::onBgaSpinCountChanged(int) {
}
void MainWindow::onBgaBtnStartClicked() {
    appendLog(QStringLiteral("BGA 测量功能开发中"));
}
void MainWindow::onBgaBtnPlotClicked() {
    appendLog(QStringLiteral("BGA 绘图功能开发中"));
}
void MainWindow::onBgaBtnBallShowClicked() {
    appendLog(QStringLiteral("BGA 焊球显示功能开发中"));
}
void MainWindow::onBgaBtnImportClicked() {
    auto path = QFileDialog::getOpenFileName(this,
        QStringLiteral("导入 BGA 数据"), {}, "CSV (*.csv);;TXT (*.txt)");
    if (path.isEmpty()) return;
    appendLog(QStringLiteral("BGA 数据导入: ") + path);
}
void MainWindow::onBgaBtnSaveClicked() {
    appendLog(QStringLiteral("BGA 导出功能开发中"));
}

// ============================================================
// qfpWidget
// ============================================================

void MainWindow::onQfpSpinCountChanged(int) {
}
void MainWindow::onQfpBtnStartClicked() {
    appendLog(QStringLiteral("QFP 测量功能开发中"));
}
void MainWindow::onQfpBtnPlotClicked() {
    appendLog(QStringLiteral("QFP 绘图功能开发中"));
}
void MainWindow::onQfpBtnPinShowClicked() {
    appendLog(QStringLiteral("QFP 引脚显示功能开发中"));
}
void MainWindow::onQfpBtnImportClicked() {
    auto path = QFileDialog::getOpenFileName(this,
        QStringLiteral("导入 QFP 数据"), {}, "CSV (*.csv);;TXT (*.txt)");
    if (path.isEmpty()) return;
    appendLog(QStringLiteral("QFP 数据导入: ") + path);
}
void MainWindow::onQfpBtnSaveClicked() {
    appendLog(QStringLiteral("QFP 导出功能开发中"));
}
