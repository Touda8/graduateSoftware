#pragma once


#include <QMainWindow>
#include <QMenu>
#include <memory>
#include "common/interfaces.h"
#include "reconstruction/CalibLoader.h"
#include "reconstruction/ReconstructionPipeline.h"
#include "pointcloud/PointCloudIO.h"
#include "pointcloud/PointCloudFilter.h"
#include "measurement/BGADetector.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <thread>
#include <map>

// forward declarations
namespace Ui { class twoProjectorClass; }
namespace tp { class VtkWidget; class ZoomableImageWidget; }
class ProjectorManager;
class QListWidgetItem;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    // 重建状态机
    enum class State { IDLE, CALIB_LOADING, READY, RECONSTRUCTING, DONE };

    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() noexcept override;

signals:
    // 日志追加（可从工作线程 emit）
    void logMessage(const QString& msg);
    void bgaLogMessage(const QString& msg);
    void qfpLogMessage(const QString& msg);
    // 进度更新（可从工作线程 emit）
    void progressUpdated(int percent);

private slots:
    // ---- resWidget: 文件设置按钮 ----
    void onResBtnCalPro1BrowseClicked();
    void onResBtnScanPro1BrowseClicked();
    void onResBtnScanPro2BrowseClicked();
    void onResBtnEpiBrowseClicked();
    void onResBtnSavePathBrowseClicked();
    void onResBtnMultiRootBrowseClicked();
    void onResBtnMultiSavePathBrowseClicked();
    void onResBtnTogglePanelClicked();

    // ---- resWidget: 重建控制按钮 ----
    void onResBtnLoadCalibClicked();
    void onResBtnStartRebuildClicked();
    void onResBtnSaveCloudClicked();

    // ---- resWidget: 重建参数变更 ----
    void onResComboDecodeTypeChanged(int index);
    void onResCheckOrderedChanged(int state);
    void onResSpinFreq1Changed(int val);
    void onResSpinFreq2Changed(int val);
    void onResSpinFreq3Changed(int val);
    void onResSpinShift1Changed(int val);
    void onResSpinShift2Changed(int val);
    void onResSpinShift3Changed(int val);
    void onResDblSpinModThreshChanged(double val);
    void onResCheckDepthRangeChanged(int state);
    void onResDblSpinEpiThreshChanged(double val);
    void onResDblSpinCentroidThreshChanged(double val);
    void onResDblSpinTmaxChanged(double val);
    void onResDblSpinNumStableChanged(double val);
    void onResDblSpinFSNRHighChanged(double val);
    void onResDblSpinFSNRLowChanged(double val);
    void onResDblSpinModHighChanged(double val);
    void onResDblSpinModLowChanged(double val);
    void onResComboSaveModeChanged(int index);
    void onResSpinRepeatCountChanged(int val);
    void onResCheckGridChanged(int state);
    void onResCheckWireframeChanged(int state);
    void onResCheckInvertChanged(int state);

    // ---- 点云处理按钮 ----
    void onPcBtnImportCloudClicked();
    void onPcCloudListContextMenu(const QPoint& pos);
    void onPcCloudListItemChanged(QListWidgetItem* item);
    void onPcCloudListCurrentChanged(QListWidgetItem* current, QListWidgetItem* prev);
    void onPcCheckHeightColorChanged(int state);
    void onPcBtnCropROIClicked();
    void onPcBtnCropPlaneClicked();
    void onPcBtnSubsampleClicked();
    void onPcBtnMergeClicked();
    void onPcBtnDuplicateClicked();
    void onPcBtnSORClicked();
    void onPcBtnRORClicked();
    void onPcBtnPassThroughClicked();
    void onPcBtnGaussianClicked();
    void onPcBtnFitPlaneClicked();
    void onPcBtnNormalClicked();
    void onPcBtnCurvatureClicked();
    void onPcBtnRoughnessClicked();
    void onPcBtnICPClicked();
    void onPcBtnDistanceClicked();
    void onPcBtnPCAAlignClicked();
    void onPcBtnCenterClicked();
    void onPcBtnLevelPlaneClicked();
    void onPcBtnApplyTransformClicked();
    void onPcBtnViewFrontClicked();
    void onPcBtnViewBackClicked();
    void onPcBtnViewLeftClicked();
    void onPcBtnViewRightClicked();
    void onPcBtnViewTopClicked();
    void onPcBtnViewBottomClicked();
    void onPcBtnViewIsometricClicked();
    void onPcBtnViewResetClicked();
    void onPcBtnExportPLYClicked();
    void onPcBtnExportPCDClicked();
    void onPcBtnExportCSVClicked();
    void onPcBtnScreenshotClicked();

    // ---- 点云参数控件 ----
    void onPcSpinVoxelSizeChanged(double val);
    void onPcSpinSORKChanged(int val);
    void onPcSpinSORStdChanged(double val);
    void onPcSpinRORRadiusChanged(double val);
    void onPcSpinRORMinChanged(int val);
    void onPcComboPassAxisChanged(int index);
    void onPcComboFitMethodChanged(int index);
    void onPcSpinNormalRChanged(double val);
    void onPcSpinRotXChanged(double val);
    void onPcSpinRotYChanged(double val);
    void onPcSpinRotZChanged(double val);
    void onPcSpinTransXChanged(double val);
    void onPcSpinTransYChanged(double val);
    void onPcSpinTransZChanged(double val);

    // ---- ROI crop ----
    void performROICrop();

    // ---- bgaWidget ----
    void onBgaSpinCountChanged(int val);
    void onBgaBtnStartClicked();
    void onBgaBtnPlotClicked();
    void onBgaBtnBallShowClicked();
    void onBgaBtnImportClicked();
    void onBgaBtnSaveClicked();
    void onBgaBtnImgFolderBrowseClicked();
    void onBgaBtnCloudFolderBrowseClicked();

    // ---- qfpWidget ----
    void onQfpSpinCountChanged(int val);
    void onQfpBtnStartClicked();
    void onQfpBtnPlotClicked();
    void onQfpBtnPinShowClicked();
    void onQfpBtnImportClicked();
    void onQfpBtnSaveClicked();
    void onQfpBtnImgFolderBrowseClicked();
    void onQfpBtnCloudFolderBrowseClicked();

    // ---- capWidget: 投影仪控制 ----
    void updateProStatusLabel(int projIdx, bool connected);
    void appendCapLog(const QString& msg);

    // ---- 内部辅助 ----
    void appendLog(const QString& msg);
    void appendBgaLog(const QString& msg);
    void appendQfpLog(const QString& msg);
    void setState(State s);

private:
    void setupConnections();
    void initVtkWidget();
    void setupProjectorManager();
    void startSingleReconstruction();
    void startMultiReconstruction();
    void updateBgaChartsFromCurrentResults();

    // Multi-cloud helpers
    std::string generateCloudId();
    tp::CloudEntry* selectedEntry();

    Ui::twoProjectorClass* ui;
    State state_ = State::IDLE;
    tp::VtkWidget* vtkWidget_ = nullptr;
    ProjectorManager* projMgr_ = nullptr;
    bool continuousRunning_ = false;
    tp::ReconParams reconParams_;
    tp::DualCalibData dualCalib_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud_;
    tp::PointCloudFilter filter_;
    std::vector<double> bgaCoplanarities_;

    // Multi-cloud management
    std::map<std::string, tp::CloudEntry> cloudStore_;
    std::string selectedCloudId_;
    int cloudIdCounter_ = 0;

    // ZoomableImageWidget replacements for QLabels
    tp::ZoomableImageWidget* zoomBgaBar_ = nullptr;
    tp::ZoomableImageWidget* zoomBgaLine_ = nullptr;
    tp::ZoomableImageWidget* zoomBgaBall_ = nullptr;
    tp::ZoomableImageWidget* zoomResPhase1_ = nullptr;
    tp::ZoomableImageWidget* zoomResPhase2_ = nullptr;
    tp::ZoomableImageWidget* zoomCapShow_ = nullptr;
    tp::ZoomableImageWidget* zoomQfpSeg_ = nullptr;
    tp::ZoomableImageWidget* zoomQfpCoplanar_ = nullptr;
    tp::ZoomableImageWidget* zoomQfpMaxWarpAngle_ = nullptr;
    tp::ZoomableImageWidget* zoomQfpAvgWarpAngle_ = nullptr;
    tp::ZoomableImageWidget* zoomQfpWarpHeight_ = nullptr;

    // BGA results
    std::vector<tp::BallResult> lastBgaResults_;
    cv::Mat lastBgaImage_;
    Eigen::Vector3d lastSubstrateNormal_{0, 0, -1};
    double lastSubstrateD_ = 0;
    int lastBgaNumRows_ = 0;

    // QFP chart cache
    std::vector<double> qfpCoplanaritySeries_;
    std::vector<double> qfpMaxWarpAngleSeries_;
    std::vector<double> qfpAvgWarpAngleSeries_;
    std::vector<double> qfpMaxWarpHeightSeries_;
    cv::Mat lastQfpSegImage_;
};
