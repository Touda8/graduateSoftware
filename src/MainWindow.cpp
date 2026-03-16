#include "MainWindow.h"
#include "ui_twoProjector.h"
#include "visualization/VtkWidget.h"
#include "visualization/ChartRenderer.h"
#include "visualization/ZoomableImageWidget.h"
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
#include <QHBoxLayout>
#include <QLabel>
#include <QFileInfo>
#include <QListWidgetItem>
#include <QCheckBox>
#include <QDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QRadioButton>
#include <QButtonGroup>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QMetaType>
#include <QImage>
#include <QPixmap>
#include <QRegularExpression>
#include <QPointer>
#include <QTimer>
#include <QStorageInfo>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <future>
#include <numeric>
#include <mutex>
#include <atomic>
#include "measurement/BGADetector.h"
#include "measurement/BGAMeasurePipeline.h"
#include "algorithm/picsrc/chart_bindgen.hpp"
#include "F:/project/Envlib/daheng/GalaxySDK/Samples/C++ SDK/inc/GalaxyIncludes.h"

namespace {
using namespace GxIAPICPP;

class DahengCameraService {
public:
    ~DahengCameraService() {
        QString dummy;
        close(dummy);
        if (inited_) {
            try { IGXFactory::GetInstance().Uninit(); }
            catch (...) {}
            inited_ = false;
        }
    }

    bool isOpen() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return opened_;
    }

    bool isCapturing() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return grabbing_;
    }

    bool queryInfo(QString& sn, int& width, int& height, QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!opened_) { err = QStringLiteral("相机未连接"); return false; }
        try {
            sn = QString::fromLocal8Bit(dev_->GetDeviceInfo().GetSN().c_str());
            width = static_cast<int>(remote_->GetIntFeature("Width")->GetValue());
            height = static_cast<int>(remote_->GetIntFeature("Height")->GetValue());
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("读取相机信息失败: %1").arg(e.what());
            return false;
        }
    }

    bool scan(QStringList& snList, QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!ensureInitUnlocked(err)) return false;
        try {
            devices_.clear();
            IGXFactory::GetInstance().UpdateDeviceList(1000, devices_);
            snList.clear();
            for (auto& d : devices_) snList.push_back(QString::fromLocal8Bit(d.GetSN().c_str()));
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("扫描失败: %1").arg(e.what());
            return false;
        } catch (const std::exception& e) {
            err = QStringLiteral("扫描异常: %1").arg(e.what());
            return false;
        }
    }

    bool open(const QString& preferredSn, QString& actualSn, QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!ensureInitUnlocked(err)) return false;
        try {
            devices_.clear();
            IGXFactory::GetInstance().UpdateDeviceList(1000, devices_);
            if (devices_.empty()) {
                err = QStringLiteral("未检测到相机设备");
                return false;
            }

            if (opened_) closeUnlocked();

            QString picked = preferredSn.trimmed();
            if (picked.isEmpty()) picked = QString::fromLocal8Bit(devices_[0].GetSN().c_str());

            bool found = false;
            for (auto& d : devices_) {
                if (QString::fromLocal8Bit(d.GetSN().c_str()) == picked) {
                    found = true;
                    break;
                }
            }
            if (!found) picked = QString::fromLocal8Bit(devices_[0].GetSN().c_str());

            dev_ = IGXFactory::GetInstance().OpenDeviceBySN(picked.toStdString().c_str(), GX_ACCESS_EXCLUSIVE);
            remote_ = dev_->GetRemoteFeatureControl();
            local_ = dev_->GetFeatureControl();
            opened_ = true;
            actualSn = QString::fromLocal8Bit(dev_->GetDeviceInfo().GetSN().c_str());

            // 默认配置
            remote_->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");
            remote_->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
            remote_->GetEnumFeature("TriggerMode")->SetValue("Off");
            if (remote_->IsImplemented("ExposureAuto"))
                remote_->GetEnumFeature("ExposureAuto")->SetValue("Off");
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("连接失败: %1").arg(e.what());
            return false;
        } catch (const std::exception& e) {
            err = QStringLiteral("连接异常: %1").arg(e.what());
            return false;
        }
    }

    bool close(QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        Q_UNUSED(err);
        closeUnlocked();
        return true;
    }

    bool startCapture(QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!opened_) { err = QStringLiteral("相机未连接"); return false; }
        return ensureStreamUnlocked(err) && ensureGrabUnlocked(err);
    }

    bool stopCapture(QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        Q_UNUSED(err);
        try {
            if (grabbing_) {
                remote_->GetCommandFeature("AcquisitionStop")->Execute();
                stream_->StopGrab();
                grabbing_ = false;
            }
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("停止采集失败: %1").arg(e.what());
            return false;
        }
    }

    bool applyTrigger(const QString& mode, const QString& source, QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!opened_) { err = QStringLiteral("相机未连接"); return false; }
        try {
            remote_->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
            const bool on = mode.compare("On", Qt::CaseInsensitive) == 0;
            remote_->GetEnumFeature("TriggerMode")->SetValue(on ? "On" : "Off");
            if (on) {
                QString src = source.trimmed();
                if (src.compare("Software", Qt::CaseInsensitive) == 0) remote_->GetEnumFeature("TriggerSource")->SetValue("Software");
                else if (src.compare("Line0", Qt::CaseInsensitive) == 0) remote_->GetEnumFeature("TriggerSource")->SetValue("Line0");
                else if (src.compare("Line1", Qt::CaseInsensitive) == 0) remote_->GetEnumFeature("TriggerSource")->SetValue("Line1");
                else if (src.compare("Line2", Qt::CaseInsensitive) == 0) remote_->GetEnumFeature("TriggerSource")->SetValue("Line2");
                else remote_->GetEnumFeature("TriggerSource")->SetValue("Software");
            }
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("触发配置失败: %1").arg(e.what());
            return false;
        }
    }

    bool setExposureUs(double exposureUs, QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!opened_) { err = QStringLiteral("相机未连接"); return false; }
        try {
            if (exposureUs < 10.0) exposureUs = 10.0;
            remote_->GetFloatFeature("ExposureTime")->SetValue(exposureUs);
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("设置曝光失败: %1").arg(e.what());
            return false;
        }
    }

    bool setFrameRateHz(double hz, QString& err) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!opened_) { err = QStringLiteral("相机未连接"); return false; }
        try {
            if (!remote_->IsImplemented("AcquisitionFrameRate")) return true;
            if (remote_->IsImplemented("AcquisitionFrameRateMode"))
                remote_->GetEnumFeature("AcquisitionFrameRateMode")->SetValue("On");
            if (hz > 0.1) remote_->GetFloatFeature("AcquisitionFrameRate")->SetValue(hz);
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("设置帧率失败: %1").arg(e.what());
            return false;
        }
    }

    bool captureOne(const QString& mode, const QString& source, cv::Mat& outMat, QString& err, int timeoutMs = 1500) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!opened_) { err = QStringLiteral("相机未连接"); return false; }
        if (!ensureStreamUnlocked(err)) return false;
        if (!ensureGrabUnlocked(err)) return false;

        try {
            const bool trigOn = mode.compare("On", Qt::CaseInsensitive) == 0;
            if (trigOn && source.compare("Software", Qt::CaseInsensitive) == 0) {
                remote_->GetCommandFeature("TriggerSoftware")->Execute();
            }

            CImageDataPointer frame = stream_->GetImage(timeoutMs);
            if (frame->GetStatus() != GX_FRAME_STATUS_SUCCESS) {
                err = QStringLiteral("取帧超时或失败（请检查触发源/连线）");
                return false;
            }

            const uint64_t w = frame->GetWidth();
            const uint64_t h = frame->GetHeight();
            void* pData = nullptr;
            if (frame->GetPixelFormat() == GX_PIXEL_FORMAT_MONO8) {
                pData = frame->ConvertToRaw8(GX_BIT_0_7);
                cv::Mat gray((int)h, (int)w, CV_8UC1, pData);
                outMat = gray.clone();
            } else {
                pData = frame->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true);
                cv::Mat rgb((int)h, (int)w, CV_8UC3, pData);
                cv::cvtColor(rgb, outMat, cv::COLOR_RGB2BGR);
            }
            return !outMat.empty();
        } catch (CGalaxyException& e) {
            err = QStringLiteral("采集失败: %1").arg(e.what());
            return false;
        } catch (const std::exception& e) {
            err = QStringLiteral("采集异常: %1").arg(e.what());
            return false;
        }
    }

private:
    bool ensureInitUnlocked(QString& err) {
        try {
            if (!inited_) {
                IGXFactory::GetInstance().Init();
                inited_ = true;
            }
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("SDK初始化失败: %1").arg(e.what());
            return false;
        }
    }

    bool ensureStreamUnlocked(QString& err) {
        try {
            if (!streamOpened_) {
                uint32_t streamCount = dev_->GetStreamCount();
                if (streamCount == 0) {
                    err = QStringLiteral("设备无可用数据流");
                    return false;
                }
                stream_ = dev_->OpenStream(0);
                streamOpened_ = true;
            }
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("打开数据流失败: %1").arg(e.what());
            return false;
        }
    }

    bool ensureGrabUnlocked(QString& err) {
        try {
            if (!grabbing_) {
                stream_->StartGrab();
                remote_->GetCommandFeature("AcquisitionStart")->Execute();
                grabbing_ = true;
            }
            return true;
        } catch (CGalaxyException& e) {
            err = QStringLiteral("启动采集失败: %1").arg(e.what());
            return false;
        }
    }

    void closeUnlocked() {
        try {
            if (grabbing_) {
                remote_->GetCommandFeature("AcquisitionStop")->Execute();
                stream_->StopGrab();
                grabbing_ = false;
            }
        } catch (...) {}
        try {
            if (streamOpened_) {
                stream_->Close();
                streamOpened_ = false;
            }
        } catch (...) {}
        try {
            if (opened_) {
                dev_->Close();
                opened_ = false;
            }
        } catch (...) {}
        try { stream_ = CGXStreamPointer(); } catch (...) {}
        try { remote_ = CGXFeatureControlPointer(); } catch (...) {}
        try { local_ = CGXFeatureControlPointer(); } catch (...) {}
        try { dev_ = CGXDevicePointer(); } catch (...) {}
        devices_.clear();
    }

    mutable std::mutex mtx_;
    bool inited_ = false;
    bool opened_ = false;
    bool streamOpened_ = false;
    bool grabbing_ = false;
    gxdeviceinfo_vector devices_;
    CGXDevicePointer dev_;
    CGXFeatureControlPointer remote_;
    CGXFeatureControlPointer local_;
    CGXStreamPointer stream_;
};

// XYZ 点云转为 XYZRGB（默认白色），用于导入/滤波后赋给 currentCloud_
pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzToRgb(
    pcl::PointCloud<pcl::PointXYZ>::Ptr src)
{
    auto dst = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    dst->reserve(src->size());
    for (const auto& p : src->points) {
        pcl::PointXYZRGB pt;
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.r = 200; pt.g = 200; pt.b = 200;
        dst->push_back(pt);
    }
    return dst;
}

// Windows 上 cv::imread 使用 ANSI 代码页，无法处理 UTF-8 中文路径。
// 通过 std::filesystem::path (wchar_t) + cv::imdecode 绕过此限制。
cv::Mat imreadSafe(const std::string& utf8Path, int flags) {
    std::ifstream ifs(std::filesystem::u8path(utf8Path), std::ios::binary);
    if (!ifs) return {};
    std::vector<uchar> buf(std::istreambuf_iterator<char>(ifs), {});
    return cv::imdecode(buf, flags);
}

bool imwriteSafe(const std::string& utf8Path, const cv::Mat& img) {
    if (img.empty()) return false;
    std::string ext = ".bmp";
    auto pos = utf8Path.find_last_of('.');
    if (pos != std::string::npos) {
        ext = utf8Path.substr(pos);
    }
    std::vector<uchar> buf;
    if (!cv::imencode(ext, img, buf)) return false;
    std::ofstream ofs(std::filesystem::u8path(utf8Path), std::ios::binary);
    if (!ofs) return false;
    ofs.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
    return static_cast<bool>(ofs);
}

// 统一的 Mat -> QImage 深拷贝转换，避免跨线程/临时数据生命周期问题
QImage matToQImageCopy(const cv::Mat& src) {
    if (src.empty()) return {};

    cv::Mat converted;
    switch (src.type()) {
    case CV_8UC1:
        cv::cvtColor(src, converted, cv::COLOR_GRAY2RGB);
        break;
    case CV_8UC3:
        cv::cvtColor(src, converted, cv::COLOR_BGR2RGB);
        break;
    case CV_8UC4:
        cv::cvtColor(src, converted, cv::COLOR_BGRA2RGBA);
        return QImage(converted.data, converted.cols, converted.rows,
            static_cast<int>(converted.step), QImage::Format_RGBA8888).copy();
    default:
        return {};
    }

    return QImage(converted.data, converted.cols, converted.rows,
        static_cast<int>(converted.step), QImage::Format_RGB888).copy();
}

QImage loadChartImage(const QString& filePath) {
    QImage img(filePath);
    return img.isNull() ? QImage() : img.copy();
}

QString ensureOutputDir() {
    QDir outDir(QDir::currentPath() + "/output");
    if (!outDir.exists()) outDir.mkpath(".");
    return outDir.absolutePath();
}

QImage renderPicSrcBgaLineChart(const std::vector<double>& coplanarities) {
    if (coplanarities.empty()) return {};
    const QString outDir = ensureOutputDir();
    const QString outFile = outDir + "/bga_line_chart.png";

    {
        GdiplusSession gdi;
        ::ChartRenderer cr(700, 400);
        cr.drawLineChart(coplanarities,
            u8"BGA共面度10次重复测量",
            u8"测量次数",
            u8"共面度/mm",
            outFile.toStdString());
    }
    return loadChartImage(outFile);
}

QImage renderPicSrcBgaDeviationBarChart(const std::vector<tp::BallResult>& balls) {
    std::vector<double> heights;
    heights.reserve(balls.size());
    for (const auto& b : balls) {
        if (b.success) heights.push_back(b.height);
    }
    if (heights.empty()) return {};

    const double meanH = std::accumulate(heights.begin(), heights.end(), 0.0)
        / static_cast<double>(heights.size());

    std::vector<double> dev;
    dev.reserve(heights.size());
    for (double h : heights) dev.push_back(h - meanH); // signed deviation

    std::vector<double> absDev(dev.size());
    for (size_t i = 0; i < dev.size(); ++i) absDev[i] = std::abs(dev[i]);
    const double mn = vec_mean(absDev);
    const double sd = vec_std(absDev);
    const double threshold = mn + 2.0 * sd;

    const QString outDir = ensureOutputDir();
    const QString outFile = outDir + "/bga_ball_deviation.png";

    {
        GdiplusSession gdi;
        ::ChartRenderer cr(700, 400);
        cr.drawThresholdBarChart(dev,
            u8"BGA焊球共面度偏差分布",
            u8"焊球编号",
            u8"共面度偏差/mm",
            threshold,
            Gdiplus::Color(76, 175, 80),
            Gdiplus::Color(255, 87, 34),
            outFile.toStdString());
    }
    return loadChartImage(outFile);
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

    // 明确注册图像相关元类型（排查跨线程图像传递问题时更稳健）
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<QImage>("QImage");
    qRegisterMetaType<QPixmap>("QPixmap");

    // 用 ZoomableImageWidget 替换 QLabel, 解决图片不显示 + 支持滚轮缩放
    // 注意：替换失败时保留原 QLabel 指针，不做置空，便于 fallback 调试
    auto replaceLabel = [](QLabel* label, tp::ZoomableImageWidget*& outWidget) -> bool {
        if (!label || !label->parentWidget()) return false;
        auto* parent = label->parentWidget();
        auto* parentLayout = parent->layout();

        auto* newWidget = new tp::ZoomableImageWidget(parent);

        if (parentLayout) {
            if (parentLayout->replaceWidget(label, newWidget)) {
                label->hide();
                outWidget = newWidget;
                return true;
            }
        }

        // fallback: 无法插入布局时，覆盖原label几何
        newWidget->setGeometry(label->geometry());
        newWidget->show();
        label->hide();
        outWidget = newWidget;
        return true;
    };

    replaceLabel(ui->bgaBarLabel,    zoomBgaBar_);
    replaceLabel(ui->bgaLineLabel,   zoomBgaLine_);
    replaceLabel(ui->bgaBallLabel,   zoomBgaBall_);
    replaceLabel(ui->resPhaseLabel1, zoomResPhase1_);
    replaceLabel(ui->resPhaseLabel2, zoomResPhase2_);
    replaceLabel(ui->capShowLabel,   zoomCapShow_);
    replaceLabel(ui->qfpSegLabel,    zoomQfpSeg_);
    replaceLabel(ui->qfpCoplanarLabel, zoomQfpCoplanar_);
    replaceLabel(ui->qfpMaxWarpAngleLabel, zoomQfpMaxWarpAngle_);
    replaceLabel(ui->qfpAvgWarpAngleLabel, zoomQfpAvgWarpAngle_);
    replaceLabel(ui->qfpWarpHeightLabel, zoomQfpWarpHeight_);

    tp::Logger::instance().info(
        std::string("Zoomable widgets init: bgaBar=") + (zoomBgaBar_ ? "ok" : "null") +
        ", bgaLine=" + (zoomBgaLine_ ? std::string("ok") : std::string("null")) +
        ", bgaBall=" + (zoomBgaBall_ ? std::string("ok") : std::string("null")) +
        ", resP1=" + (zoomResPhase1_ ? std::string("ok") : std::string("null")) +
        ", resP2=" + (zoomResPhase2_ ? std::string("ok") : std::string("null")) +
        ", cap=" + (zoomCapShow_ ? std::string("ok") : std::string("null")) +
        ", qfpSeg=" + (zoomQfpSeg_ ? std::string("ok") : std::string("null")) +
        ", qfpC=" + (zoomQfpCoplanar_ ? std::string("ok") : std::string("null")) +
        ", qfpMaxA=" + (zoomQfpMaxWarpAngle_ ? std::string("ok") : std::string("null")) +
        ", qfpAvgA=" + (zoomQfpAvgWarpAngle_ ? std::string("ok") : std::string("null")) +
        ", qfpMaxH=" + (zoomQfpWarpHeight_ ? std::string("ok") : std::string("null")));

    initVtkWidget();
    setupConnections();
    setupProjectorManager();
    tp::Config::instance().load(this);
    onResComboSaveModeChanged(ui->resComboSaveMode->currentIndex());
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
    connect(ui->resBtnScanPro1Browse, &QPushButton::clicked,
            this, &MainWindow::onResBtnScanPro1BrowseClicked);
    connect(ui->resBtnScanPro2Browse, &QPushButton::clicked,
            this, &MainWindow::onResBtnScanPro2BrowseClicked);
    connect(ui->resBtnEpiBrowse, &QPushButton::clicked,
            this, &MainWindow::onResBtnEpiBrowseClicked);
    connect(ui->resBtnSavePathBrowse, &QPushButton::clicked,
            this, &MainWindow::onResBtnSavePathBrowseClicked);
    connect(ui->resBtnMultiRootBrowse, &QPushButton::clicked,
            this, &MainWindow::onResBtnMultiRootBrowseClicked);
    connect(ui->resBtnMultiSavePathBrowse, &QPushButton::clicked,
            this, &MainWindow::onResBtnMultiSavePathBrowseClicked);
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
    // 点云列表右键菜单
    ui->pcCloudListWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->pcCloudListWidget, &QWidget::customContextMenuRequested,
            this, &MainWindow::onPcCloudListContextMenu);
    // 点云列表勾选框变化
    connect(ui->pcCloudListWidget, &QListWidget::itemChanged,
            this, &MainWindow::onPcCloudListItemChanged);
    // 点云列表选中变化
    connect(ui->pcCloudListWidget, &QListWidget::currentItemChanged,
            this, &MainWindow::onPcCloudListCurrentChanged);
    // 高度染色切换
    connect(ui->pcCheckHeightColor, &QCheckBox::stateChanged,
            this, &MainWindow::onPcCheckHeightColorChanged);
    // ROI crop signal
    connect(vtkWidget_, &tp::VtkWidget::roiCropRequested,
            this, &MainWindow::performROICrop);
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

    // ---- Pro1 路径变化时自动填充 Pro2 ----
    connect(ui->resEditScanPro1, &QLineEdit::textChanged,
            this, [this](const QString& text) {
        QDir dir(text);
        QString measureDir = dir.dirName();
        if (!dir.cdUp()) return;
        QString projDir = dir.dirName();
        if (projDir == "1") {
            if (!dir.cdUp()) return;
            ui->resEditScanPro2->setText(dir.absolutePath() + "/2/" + measureDir);
        }
    });

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
    connect(ui->bgaBtnImgFolderBrowse, &QPushButton::clicked,
            this, &MainWindow::onBgaBtnImgFolderBrowseClicked);
    connect(ui->bgaBtnCloudFolderBrowse, &QPushButton::clicked,
            this, &MainWindow::onBgaBtnCloudFolderBrowseClicked);

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
    connect(ui->qfpBtnImgFolderBrowse, &QPushButton::clicked,
            this, &MainWindow::onQfpBtnImgFolderBrowseClicked);
    connect(ui->qfpBtnCloudFolderBrowse, &QPushButton::clicked,
            this, &MainWindow::onQfpBtnCloudFolderBrowseClicked);

    // ---- 内部日志/进度信号 (QueuedConnection for thread safety) ----
    connect(this, &MainWindow::logMessage,
            this, &MainWindow::appendLog, Qt::QueuedConnection);
    connect(this, &MainWindow::bgaLogMessage,
            this, &MainWindow::appendBgaLog, Qt::QueuedConnection);
    connect(this, &MainWindow::qfpLogMessage,
            this, &MainWindow::appendQfpLog, Qt::QueuedConnection);
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
    ui->resBtnStartRebuild->setEnabled(ready || done);
    ui->resBtnSaveCloud->setEnabled(done);
}

void MainWindow::appendLog(const QString& msg) {
    auto line = QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg;
    ui->resLogText->append(line);
    tp::Logger::instance().info(msg.toStdString());
}

void MainWindow::appendBgaLog(const QString& msg) {
    ui->bgaLogText->append(
        QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg);
    tp::Logger::instance().info(("BGA: " + msg).toStdString());
}

void MainWindow::appendQfpLog(const QString& msg) {
    ui->qfpLogText->append(
        QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg);
    tp::Logger::instance().info(("QFP: " + msg).toStdString());
}

void MainWindow::appendCapLog(const QString& msg) {
    ui->capLogText->append(
        QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg);
}

// ============================================================
// Multi-cloud helpers
// ============================================================

std::string MainWindow::generateCloudId() {
    return "cloud_" + std::to_string(++cloudIdCounter_);
}

tp::CloudEntry* MainWindow::selectedEntry() {
    if (selectedCloudId_.empty()) return nullptr;
    auto it = cloudStore_.find(selectedCloudId_);
    if (it == cloudStore_.end()) return nullptr;
    return &(it->second);
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
    auto camSvc = std::make_shared<DahengCameraService>();

    struct CameraRuntimeState {
        std::atomic_bool connected{ false };
        std::atomic_bool capturing{ false };
        std::atomic_bool stopLoop{ false };
        std::atomic_bool fallbackSwitching{ false };
        std::atomic_llong lastFrameMs{ 0 };
        std::atomic_int triggerSeq{ 0 };
        std::atomic_llong totalFrames{ 0 };
        std::thread captureThread;
        double fps = 0.0;
        std::mutex statMtx;
    };
    auto camState = std::make_shared<CameraRuntimeState>();
    auto trigModeOn = std::make_shared<std::atomic_bool>(ui->capComboTrigMode->currentText().compare("On", Qt::CaseInsensitive) == 0);
    auto trigSrcCode = std::make_shared<std::atomic_int>(0); // 0=Software,1=Line0,2=Line1,3=Line2

    struct AutoCaptureState {
        std::mutex mtx;
        bool enableSave = true;
        QString imageFormat = "BMP";
        int intervalMode = 0; // 0:按帧数 1:按时间
        int intervalValue = 1;
        int limitMode = 0; // 0:帧 1:秒 2:分钟
        int maxFrameLimit = 0;
        int maxTimeLimit = 0;
        int activeProjector = 0; // 0:none 1:pro1 2:pro2
        QString pro1Base;
        QString pro2Base;
        QString activeSessionDir;
        int nextImageIndex = 1;
        long long sessionStartMs = 0;
        long long savedCount = 0;
        long long sampledFrames = 0;
        long long lastSavedFrameTotal = 0;
        long long lastSavedMs = 0;
    };
    auto autoCapState = std::make_shared<AutoCaptureState>();
    struct AutoCaptureUiHandles {
        QPointer<QDialog> dlg;
        QPointer<QTextEdit> log;
        QPointer<QLabel> infoBuffer;
        QPointer<QLabel> infoDisk;
        QPointer<QLabel> infoTotal;
        QPointer<QLineEdit> pro1Edit;
        QPointer<QLineEdit> pro2Edit;
        QPointer<QComboBox> fmtCombo;
        QPointer<QSpinBox> intervalSpin;
        QPointer<QRadioButton> byFrameRadio;
        QPointer<QRadioButton> byTimeRadio;
        QPointer<QSpinBox> maxFrameSpin;
        QPointer<QSpinBox> maxTimeSpin;
        QPointer<QRadioButton> limitFrameRadio;
        QPointer<QRadioButton> limitSecRadio;
        QPointer<QRadioButton> limitMinRadio;
        QPointer<QCheckBox> saveEnableCheck;
    };
    auto autoUi = std::make_shared<AutoCaptureUiHandles>();

    auto currentMs = []() -> long long {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };

    auto appendAutoLog = [this, autoUi](const QString& msg) {
        if (!autoUi->log) return;
        const QString line = QDateTime::currentDateTime().toString("[hh:mm:ss] ") + msg;
        QMetaObject::invokeMethod(autoUi->log, [log = autoUi->log, line]() {
            if (log) log->append(line);
        }, Qt::QueuedConnection);
    };

    auto makeNextNumericSessionDir = [](const QString& baseDir) -> QString {
        QDir base(baseDir);
        if (!base.exists()) base.mkpath(".");
        const QStringList dirs = base.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
        int mx = 0;
        QRegularExpression re("^\\d+$");
        for (const auto& d : dirs) {
            if (re.match(d).hasMatch()) mx = std::max(mx, d.toInt());
        }
        const int next = mx + 1;
        const QString child = QString::number(next);
        base.mkpath(child);
        return base.absoluteFilePath(child);
    };

    auto parseNumber = [](const QString& text, double defVal) {
        QRegularExpression re("([-+]?\\d*\\.?\\d+)");
        auto m = re.match(text);
        if (!m.hasMatch()) return defVal;
        bool ok = false;
        double v = m.captured(1).toDouble(&ok);
        return ok ? v : defVal;
    };

    if (ui->capComboTrigSrc->findText("Line2", Qt::MatchExactly) < 0) {
        ui->capComboTrigSrc->addItem("Line2");
    }
    ui->capComboTrigMode->setEnabled(false); // 触发模式只读，由联动逻辑自动控制

    // 文件设置区域新增“自动采集”入口（非模态）
    auto* autoCapBtn = new QPushButton(QStringLiteral("自动采集"), ui->capFileGroupBox);
    autoCapBtn->setStyleSheet("QPushButton{background:#FFFFFF;color:#333333;border:1px solid #E5E5E5;padding:4px 10px;}"
                              "QPushButton:hover{background:#F5F5F5;}");
    if (ui->capFileRow) {
        ui->capFileRow->addWidget(autoCapBtn);
    }

    // 左右布局按 40%/60% 调整
    if (ui->capMainLayout) {
        ui->capMainLayout->setStretch(0, 4);
        ui->capMainLayout->setStretch(1, 6);
    }

    // 顶部状态栏：替换树形列表可视化
    ui->capTreeWidget->hide();
    auto* capConnLayout = ui->capConnLayout;
    auto* camStatusLabel = new QLabel(QStringLiteral("● 相机状态：未连接"), ui->capConnGroupBox);
    auto* projStatusLabel = new QLabel(QStringLiteral("● 投影仪总状态：未连接"), ui->capConnGroupBox);
    auto* trigStatusLabel = new QLabel(QStringLiteral("● 当前触发模式：软触发"), ui->capConnGroupBox);
    camStatusLabel->setStyleSheet("color:#808080;font-weight:600;");
    projStatusLabel->setStyleSheet("color:#808080;font-weight:600;");
    trigStatusLabel->setStyleSheet("color:#2F86F6;font-weight:600;");
    capConnLayout->addWidget(camStatusLabel);
    capConnLayout->addWidget(projStatusLabel);
    capConnLayout->addWidget(trigStatusLabel);

    // 相机独立按钮组（连接与采集分离）
    auto* camBtnConnect = new QPushButton(QStringLiteral("连接相机"), ui->capParamGroupBox);
    auto* camBtnDisconnect = new QPushButton(QStringLiteral("断开相机"), ui->capParamGroupBox);
    auto* camBtnStart = new QPushButton(QStringLiteral("开启相机采集"), ui->capParamGroupBox);
    auto* camBtnStop = new QPushButton(QStringLiteral("关闭相机采集"), ui->capParamGroupBox);
    auto* camRow1 = new QHBoxLayout();
    auto* camRow2 = new QHBoxLayout();
    camRow1->addWidget(camBtnConnect);
    camRow1->addWidget(camBtnDisconnect);
    camRow2->addWidget(camBtnStart);
    camRow2->addWidget(camBtnStop);
    ui->capParamGrid->addLayout(camRow1, 6, 0, 1, 2);
    ui->capParamGrid->addLayout(camRow2, 7, 0, 1, 2);

    // 右侧顶部状态
    auto* capTopStatus = new QLabel(QStringLiteral("相机分辨率：- | 帧率：0.00 fps | 累计帧数：0 | 当前触发模式：软触发"), ui->capWidget);
    capTopStatus->setAlignment(Qt::AlignCenter);
    capTopStatus->setStyleSheet("background:#F5F5F5;color:#333333;border:1px solid #D0D0D0;padding:4px;");
    ui->capRightLayout->insertWidget(0, capTopStatus);

    auto sourceCodeFromText = [](const QString& src) {
        if (src.compare("Line0", Qt::CaseInsensitive) == 0) return 1;
        if (src.compare("Line1", Qt::CaseInsensitive) == 0) return 2;
        if (src.compare("Line2", Qt::CaseInsensitive) == 0) return 3;
        return 0;
    };
    auto nowMs = []() -> long long {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };
    trigSrcCode->store(sourceCodeFromText(ui->capComboTrigSrc->currentText()));

    auto updateCameraButtons = [this, camState, camBtnConnect, camBtnDisconnect, camBtnStart, camBtnStop, camStatusLabel]() {
        const bool connected = camState->connected.load();
        const bool capturing = camState->capturing.load();

        camBtnConnect->setEnabled(!connected);
        camBtnDisconnect->setEnabled(connected);
        camBtnStart->setEnabled(connected && !capturing);
        camBtnStop->setEnabled(connected && capturing);
        ui->capBtnSoftTrigger->setEnabled(connected);

        if (!connected) {
            camStatusLabel->setText(QStringLiteral("● 相机状态：未连接"));
            camStatusLabel->setStyleSheet("color:#808080;font-weight:600;");
        } else if (capturing) {
            camStatusLabel->setText(QStringLiteral("● 相机状态：已连接（采集中）"));
            camStatusLabel->setStyleSheet("color:#00AA00;font-weight:700;");
        } else {
            camStatusLabel->setText(QStringLiteral("● 相机状态：已连接（未采集）"));
            camStatusLabel->setStyleSheet("color:#00AA00;font-weight:600;");
        }
    };
    updateCameraButtons();

    auto showPatternPreview = [this](int patternIndex) {
        cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(240, 240, 240));
        const cv::Scalar black(20, 20, 20);
        switch (patternIndex) {
        case 0: // white
            img.setTo(cv::Scalar(255, 255, 255));
            break;
        case 1: // black
            img.setTo(cv::Scalar(0, 0, 0));
            break;
        case 2: // color bars
            for (int i = 0; i < 8; ++i) {
                cv::Scalar c;
                switch (i) {
                case 0: c = cv::Scalar(0, 0, 255); break;
                case 1: c = cv::Scalar(0, 165, 255); break;
                case 2: c = cv::Scalar(0, 255, 255); break;
                case 3: c = cv::Scalar(0, 255, 0); break;
                case 4: c = cv::Scalar(255, 255, 0); break;
                case 5: c = cv::Scalar(255, 0, 0); break;
                case 6: c = cv::Scalar(255, 0, 255); break;
                default: c = cv::Scalar(255, 255, 255); break;
                }
                int x0 = i * img.cols / 8;
                int x1 = (i + 1) * img.cols / 8;
                cv::rectangle(img, cv::Rect(x0, 0, x1 - x0, img.rows), c, cv::FILLED);
            }
            break;
        case 3: // horizontal lines
            img.setTo(cv::Scalar(255, 255, 255));
            for (int y = 0; y < img.rows; y += 24)
                cv::line(img, cv::Point(0, y), cv::Point(img.cols - 1, y), black, 2);
            break;
        case 4: // vertical lines
            img.setTo(cv::Scalar(255, 255, 255));
            for (int x = 0; x < img.cols; x += 24)
                cv::line(img, cv::Point(x, 0), cv::Point(x, img.rows - 1), black, 2);
            break;
        case 5: // grid
            img.setTo(cv::Scalar(255, 255, 255));
            for (int y = 0; y < img.rows; y += 30)
                cv::line(img, cv::Point(0, y), cv::Point(img.cols - 1, y), black, 1);
            for (int x = 0; x < img.cols; x += 30)
                cv::line(img, cv::Point(x, 0), cv::Point(x, img.rows - 1), black, 1);
            break;
        case 6: // checker
            for (int y = 0; y < img.rows; y += 40)
                for (int x = 0; x < img.cols; x += 40) {
                    bool white = ((x / 40) + (y / 40)) % 2 == 0;
                    cv::rectangle(img, cv::Rect(x, y, 40, 40),
                        white ? cv::Scalar(255, 255, 255) : cv::Scalar(30, 30, 30), cv::FILLED);
                }
            break;
        default:
            break;
        }

        cv::putText(img, "Pattern Preview", cv::Point(20, 40),
            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(60, 60, 60), 2, cv::LINE_AA);
        Q_UNUSED(img);
        appendCapLog(QStringLiteral("投影图案命令已下发（右侧仅显示相机画面），图案编号: %1").arg(patternIndex));
    };

    // ---- 连接管理按钮（带禁用防重复点击） ----
    connect(ui->capBtnProjScan, &QPushButton::clicked,
            this, [this] {
                ui->capBtnProjScan->setEnabled(false);
                appendCapLog(QStringLiteral("开始扫描投影仪设备..."));
                projMgr_->scanDevices();
            });
    connect(ui->capBtnProjScan, &QPushButton::clicked,
            this, [this, camSvc] {
                std::thread([this, camSvc]() {
                    QStringList sns;
                    QString err;
                    bool ok = camSvc->scan(sns, err);
                    QMetaObject::invokeMethod(this, [this, ok, sns, err]() {
                        if (!ok) {
                            appendCapLog(QStringLiteral("相机扫描失败: ") + err);
                            return;
                        }
                        appendCapLog(QStringLiteral("相机扫描完成: 发现 %1 台").arg(sns.size()));
                        if (!sns.isEmpty()) {
                            appendCapLog(QStringLiteral("相机SN: ") + sns.join(", "));
                            if (ui->capEditSN->text().trimmed().isEmpty())
                                ui->capEditSN->setText(sns.first());
                        }
                    }, Qt::QueuedConnection);
                }).detach();
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

    // ---- 相机参数应用 ----
    connect(ui->capComboTrigMode, &QComboBox::currentTextChanged,
            this, [this, camSvc, trigModeOn, trigStatusLabel, capTopStatus, camState](const QString& txt) {
                trigModeOn->store(txt.compare("On", Qt::CaseInsensitive) == 0);
                if (!camSvc->isOpen()) return;
                QString err;
                if (!camSvc->applyTrigger(ui->capComboTrigMode->currentText(), ui->capComboTrigSrc->currentText(), err))
                    appendCapLog(QStringLiteral("触发模式设置失败: ") + err);
                else {
                    trigStatusLabel->setText(QStringLiteral("● 当前触发模式：%1/%2").arg(ui->capComboTrigMode->currentText(), ui->capComboTrigSrc->currentText()));
                    capTopStatus->setText(QStringLiteral("相机分辨率：- | 帧率：0.00 fps | 累计帧数：%1 | 当前触发模式：%2/%3")
                        .arg(camState->totalFrames.load()).arg(ui->capComboTrigMode->currentText(), ui->capComboTrigSrc->currentText()));
                }
            });
    connect(ui->capComboTrigSrc, &QComboBox::currentTextChanged,
            this, [this, camSvc, trigSrcCode, sourceCodeFromText, trigStatusLabel, capTopStatus, camState](const QString& txt) {
                trigSrcCode->store(sourceCodeFromText(txt));
                if (!camSvc->isOpen()) return;
                QString err;
                if (!camSvc->applyTrigger(ui->capComboTrigMode->currentText(), ui->capComboTrigSrc->currentText(), err))
                    appendCapLog(QStringLiteral("触发源设置失败: ") + err);
                else {
                    trigStatusLabel->setText(QStringLiteral("● 当前触发模式：%1/%2").arg(ui->capComboTrigMode->currentText(), ui->capComboTrigSrc->currentText()));
                    capTopStatus->setText(QStringLiteral("相机分辨率：- | 帧率：0.00 fps | 累计帧数：%1 | 当前触发模式：%2/%3")
                        .arg(camState->totalFrames.load()).arg(ui->capComboTrigMode->currentText(), ui->capComboTrigSrc->currentText()));
                }
            });
    connect(ui->capEditExposure, &QLineEdit::editingFinished,
            this, [this, camSvc, parseNumber] {
                if (!camSvc->isOpen()) return;
                QString err;
                double us = parseNumber(ui->capEditExposure->text(), 30000.0);
                if (!camSvc->setExposureUs(us, err))
                    appendCapLog(QStringLiteral("曝光时间设置失败: ") + err);
                else
                    appendCapLog(QStringLiteral("曝光时间已设置: %1 us").arg(us, 0, 'f', 2));
            });
    connect(ui->capEditFramerate, &QLineEdit::editingFinished,
            this, [this, camSvc, parseNumber] {
                if (!camSvc->isOpen()) return;
                QString err;
                double hz = parseNumber(ui->capEditFramerate->text(), 20.0);
                if (!camSvc->setFrameRateHz(hz, err))
                    appendCapLog(QStringLiteral("采集帧率设置失败: ") + err);
                else
                    appendCapLog(QStringLiteral("采集帧率已设置: %1 Hz").arg(hz, 0, 'f', 2));
            });

    // ---- 相机专用按钮（连接/采集解耦） ----
    connect(camBtnConnect, &QPushButton::clicked, this, [this, camSvc, camState, parseNumber,
            updateCameraButtons, trigStatusLabel, capTopStatus]() {
        const QString snText = ui->capEditSN->text().trimmed();
        const double exposureUs = parseNumber(ui->capEditExposure->text(), 30000.0);
        const double frameRate = parseNumber(ui->capEditFramerate->text(), 20.0);
        const QString trigMode = ui->capComboTrigMode->currentText();
        const QString trigSrc = ui->capComboTrigSrc->currentText();
        std::thread([this, camSvc, camState, snText, exposureUs, frameRate, trigMode, trigSrc,
                     updateCameraButtons, trigStatusLabel, capTopStatus]() {
            QString actualSn, err;
            bool ok = camSvc->open(snText, actualSn, err);
            if (ok) {
                camSvc->setExposureUs(exposureUs, err);
                camSvc->setFrameRateHz(frameRate, err);
                camSvc->applyTrigger(trigMode, trigSrc, err);
            }
            QMetaObject::invokeMethod(this, [this, ok, actualSn, err, camState,
                    updateCameraButtons, trigStatusLabel, capTopStatus, trigMode, trigSrc]() {
                camState->connected.store(ok);
                camState->capturing.store(false);
                updateCameraButtons();
                if (ok) {
                    camState->totalFrames.store(0);
                    camState->lastFrameMs.store(0);
                    camState->triggerSeq.store(0);
                    ui->capEditSN->setText(actualSn);
                    appendCapLog(QStringLiteral("相机连接成功, SN=%1").arg(actualSn));
                    trigStatusLabel->setText(QStringLiteral("● 当前触发模式：%1/%2").arg(trigMode, trigSrc));
                    capTopStatus->setText(QStringLiteral("相机分辨率：- | 帧率：0.00 fps | 累计帧数：%1 | 当前触发模式：%2/%3")
                        .arg(camState->totalFrames.load()).arg(trigMode, trigSrc));
                } else {
                    appendCapLog(QStringLiteral("相机连接失败: ") + err);
                }
            }, Qt::QueuedConnection);
        }).detach();
    });

    connect(camBtnDisconnect, &QPushButton::clicked, this, [this, camSvc, camState,
            updateCameraButtons, trigStatusLabel, capTopStatus]() {
        camState->stopLoop.store(true);
        camState->capturing.store(false);
        if (camState->captureThread.joinable()) camState->captureThread.join();
        std::thread([this, camSvc, camState, updateCameraButtons, trigStatusLabel, capTopStatus]() {
            QString err;
            camSvc->stopCapture(err);
            camSvc->close(err);
            QMetaObject::invokeMethod(this, [this, camState, updateCameraButtons, trigStatusLabel, capTopStatus]() {
                camState->connected.store(false);
                camState->capturing.store(false);
                updateCameraButtons();
                trigStatusLabel->setText(QStringLiteral("● 当前触发模式：软触发"));
                capTopStatus->setText(QStringLiteral("相机分辨率：- | 帧率：0.00 fps | 累计帧数：%1 | 当前触发模式：软触发")
                    .arg(camState->totalFrames.load()));
                if (zoomCapShow_) zoomCapShow_->clearImage();
                appendCapLog(QStringLiteral("相机已断开"));
            }, Qt::QueuedConnection);
        }).detach();
    });

            connect(camBtnStart, &QPushButton::clicked, this, [this, camSvc, camState,
                updateCameraButtons, capTopStatus, trigStatusLabel, trigModeOn, trigSrcCode, nowMs,
                autoCapState, appendAutoLog, currentMs]() {
        if (!camState->connected.load()) return;
        camState->stopLoop.store(false);
        QString err;
        if (!camSvc->startCapture(err)) {
            appendCapLog(QStringLiteral("开启采集失败: ") + err);
            return;
        }

        camState->capturing.store(true);
        updateCameraButtons();
        appendCapLog(QStringLiteral("相机采集已开启"));

        if (camState->captureThread.joinable()) camState->captureThread.join();
        camState->captureThread = std::thread([this, camSvc, camState, capTopStatus, trigStatusLabel, trigModeOn, trigSrcCode, nowMs,
                               autoCapState, appendAutoLog, currentMs]() {
            auto t0 = std::chrono::steady_clock::now();
            int frameCount = 0;
            int noFrameTick = 0;
            while (!camState->stopLoop.load()) {
                cv::Mat frame;
                QString err;
                const QString trigMode = trigModeOn->load() ? QStringLiteral("On") : QStringLiteral("Off");
                const int srcCode = trigSrcCode->load();
                const QString trigSrc = (srcCode == 1) ? QStringLiteral("Line0") :
                                        (srcCode == 2) ? QStringLiteral("Line1") :
                                        (srcCode == 3) ? QStringLiteral("Line2") : QStringLiteral("Software");
                if (camSvc->captureOne(trigMode, trigSrc, frame, err, 300)) {
                    ++frameCount;
                    const long long total = camState->totalFrames.fetch_add(1) + 1;
                    camState->lastFrameMs.store(nowMs());
                    noFrameTick = 0;

                    // 自动存图：仅硬触发（On + Line0）且激活了Pro路径时执行
                    if (trigMode == QStringLiteral("On") && trigSrc == QStringLiteral("Line0")) {
                        QString savePath;
                        QString saveExt;
                        bool shouldSave = false;
                        bool hitLimit = false;
                        {
                            std::lock_guard<std::mutex> lk(autoCapState->mtx);
                            if (autoCapState->enableSave && autoCapState->activeProjector != 0 && !autoCapState->activeSessionDir.isEmpty()) {
                                const long long elapsedMs = std::max<long long>(0, currentMs() - autoCapState->sessionStartMs);
                                bool limitOk = true;
                                if (autoCapState->limitMode == 0 && autoCapState->maxFrameLimit > 0) {
                                    limitOk = autoCapState->savedCount < autoCapState->maxFrameLimit;
                                } else if (autoCapState->limitMode == 1 && autoCapState->maxTimeLimit > 0) {
                                    limitOk = elapsedMs < (static_cast<long long>(autoCapState->maxTimeLimit) * 1000);
                                } else if (autoCapState->limitMode == 2 && autoCapState->maxTimeLimit > 0) {
                                    limitOk = elapsedMs < (static_cast<long long>(autoCapState->maxTimeLimit) * 60 * 1000);
                                }

                                if (!limitOk) {
                                    hitLimit = true;
                                    autoCapState->activeProjector = 0;
                                } else {
                                    autoCapState->sampledFrames++;
                                    bool intervalOk = true;
                                    if (autoCapState->intervalMode == 0) {
                                        const int iv = std::max(1, autoCapState->intervalValue);
                                        intervalOk = (autoCapState->sampledFrames % iv) == 0;
                                    } else {
                                        const int iv = std::max(1, autoCapState->intervalValue);
                                        intervalOk = (autoCapState->lastSavedMs == 0) || ((currentMs() - autoCapState->lastSavedMs) >= (static_cast<long long>(iv) * 1000));
                                    }

                                    if (intervalOk) {
                                        saveExt = autoCapState->imageFormat.trimmed().toLower();
                                        if (saveExt != "bmp" && saveExt != "png" && saveExt != "jpg" && saveExt != "jpeg") {
                                            saveExt = "bmp";
                                        }
                                        savePath = QDir(autoCapState->activeSessionDir)
                                            .absoluteFilePath(QString("%1.%2").arg(autoCapState->nextImageIndex++).arg(saveExt));
                                        shouldSave = true;
                                    }
                                }
                            }
                        }

                        if (hitLimit) {
                            appendAutoLog(QStringLiteral("达到自动采集限制，停止当前Pro路径自动存图"));
                        }
                        if (shouldSave && !savePath.isEmpty()) {
                            const bool ok = imwriteSafe(savePath.toStdString(), frame);
                            if (ok) {
                                {
                                    std::lock_guard<std::mutex> lk(autoCapState->mtx);
                                    autoCapState->savedCount++;
                                    autoCapState->lastSavedMs = currentMs();
                                    autoCapState->lastSavedFrameTotal = total;
                                }
                                appendAutoLog(savePath + QStringLiteral(" 保存成功"));
                            } else {
                                appendAutoLog(savePath + QStringLiteral(" 保存失败"));
                            }
                        }
                    }

                    auto now = std::chrono::steady_clock::now();
                    double sec = std::chrono::duration<double>(now - t0).count();
                    double fps = sec > 0.0 ? (frameCount / sec) : 0.0;
                    QImage qimg = matToQImageCopy(frame);

                    QString sn; int w = 0, h = 0; QString infoErr;
                    camSvc->queryInfo(sn, w, h, infoErr);

                    QMetaObject::invokeMethod(this, [this, qimg, fps, w, h, trigMode, trigSrc, capTopStatus, trigStatusLabel, total]() {
                        if (!qimg.isNull() && zoomCapShow_) {
                            zoomCapShow_->setImage(qimg);
                        }
                        capTopStatus->setText(QStringLiteral("相机分辨率：%1×%2 | 帧率：%3 fps | 累计帧数：%4 | 当前触发模式：%5/%6")
                            .arg(w).arg(h).arg(fps, 0, 'f', 2).arg(total).arg(trigMode, trigSrc));
                        trigStatusLabel->setText(QStringLiteral("● 当前触发模式：%1/%2").arg(trigMode, trigSrc));
                    }, Qt::QueuedConnection);
                } else {
                    ++noFrameTick;
                    if (noFrameTick % 4 == 0) {
                        QMetaObject::invokeMethod(this, [capTopStatus, trigMode, trigSrc, camState]() {
                            QString t = capTopStatus->text();
                            Q_UNUSED(t);
                            capTopStatus->setText(QStringLiteral("相机分辨率：- | 帧率：0.00 fps | 累计帧数：%1 | 当前触发模式：%2/%3")
                                .arg(camState->totalFrames.load()).arg(trigMode, trigSrc));
                        }, Qt::QueuedConnection);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }
        });
    });

    connect(camBtnStop, &QPushButton::clicked, this, [this, camSvc, camState, updateCameraButtons]() {
        camState->stopLoop.store(true);
        if (camState->captureThread.joinable()) camState->captureThread.join();
        QString err;
        camSvc->stopCapture(err);
        camState->capturing.store(false);
        updateCameraButtons();
        appendCapLog(QStringLiteral("相机采集已关闭"));
    });

    // 单帧采集：仅保存，不刷新UI
    connect(ui->capBtnSoftTrigger, &QPushButton::clicked, this, [this, camSvc, camState, parseNumber, capTopStatus] {
        if (!camState->connected.load()) {
            appendCapLog(QStringLiteral("请先连接相机"));
            return;
        }
        const double exposureUs = parseNumber(ui->capEditExposure->text(), 30000.0);
        const double frameRate = parseNumber(ui->capEditFramerate->text(), 20.0);
        const QString trigMode = ui->capComboTrigMode->currentText();
        const QString trigSrc = ui->capComboTrigSrc->currentText();
        const QString folderText = ui->capEditFolder->text();
        const QString sn = ui->capEditSN->text().trimmed();

        std::thread([this, camSvc, camState, exposureUs, frameRate, trigMode, trigSrc, folderText, sn, capTopStatus]() {
            QString err;
            camSvc->setExposureUs(exposureUs, err);
            camSvc->setFrameRateHz(frameRate, err);
            camSvc->applyTrigger(trigMode, trigSrc, err);
            cv::Mat frame;
            if (!camSvc->captureOne(trigMode, trigSrc, frame, err, 1000)) {
                QMetaObject::invokeMethod(this, [this, err]() {
                    appendCapLog(QStringLiteral("单帧采集失败: ") + err);
                }, Qt::QueuedConnection);
                return;
            }
            QImage qimg = matToQImageCopy(frame);
            const long long total = camState->totalFrames.fetch_add(1) + 1;
            QString infoErr, infoSn; int w = 0, h = 0;
            camSvc->queryInfo(infoSn, w, h, infoErr);
            QString folder = folderText.trimmed();
            if (folder.isEmpty()) folder = QDir::currentPath() + "/data/capture";
            QDir d(folder);
            if (!d.exists()) d.mkpath(".");
            static std::atomic<int> seq{ 0 };
            const int idx = ++seq;
            const QString usedSn = sn.isEmpty() ? QStringLiteral("UnknownSN") : sn;
            QString savePath = d.absoluteFilePath(
                QString("%1_%2_%3.jpg").arg(usedSn).arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz")).arg(idx));
            if (!qimg.isNull()) qimg.save(savePath);
            QMetaObject::invokeMethod(this, [this, savePath, capTopStatus, total, w, h, trigMode, trigSrc]() {
                capTopStatus->setText(QStringLiteral("相机分辨率：%1×%2 | 帧率：0.00 fps | 累计帧数：%3 | 当前触发模式：%4/%5")
                    .arg(w).arg(h).arg(total).arg(trigMode, trigSrc));
                appendCapLog(QStringLiteral("单帧采集-保存路径: ") + savePath);
            }, Qt::QueuedConnection);
        }).detach();
    });

    connect(this, &QObject::destroyed, this, [camState, camSvc]() {
        camState->stopLoop.store(true);
        camState->capturing.store(false);
        if (camState->captureThread.joinable()) camState->captureThread.join();
        QString err;
        camSvc->stopCapture(err);
        camSvc->close(err);
    });

    // ---- 状态反馈信号 ----
    connect(projMgr_, &ProjectorManager::connectionStateChanged,
            this, [this, projStatusLabel](int idx, bool connected) {
                updateProStatusLabel(idx, connected);
                bool p1 = projMgr_->isConnected(0);
                bool p2 = projMgr_->isConnected(1);
                const bool any = p1 || p2;
                projStatusLabel->setText(QStringLiteral("● 投影仪总状态：%1（Pro1:%2 Pro2:%3）")
                    .arg(any ? QStringLiteral("已连接") : QStringLiteral("未连接"))
                    .arg(p1 ? QStringLiteral("已连接") : QStringLiteral("未连接"))
                    .arg(p2 ? QStringLiteral("已连接") : QStringLiteral("未连接")));
                projStatusLabel->setStyleSheet(any ? "color:#00AA00;font-weight:600;" : "color:#808080;font-weight:600;");
            }, Qt::QueuedConnection);
    connect(projMgr_, &ProjectorManager::statusMessage,
            this, &MainWindow::appendCapLog,
            Qt::QueuedConnection);
    connect(projMgr_, &ProjectorManager::errorOccurred,
            this, [this](int, QString err) { appendCapLog(QStringLiteral("错误: ") + err); },
            Qt::QueuedConnection);
    connect(projMgr_, &ProjectorManager::deviceListUpdated,
            this, [this](QStringList list) {
                appendCapLog(QStringLiteral("投影仪设备列表: ") + list.join(", "));
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
    connect(ui->capBtnProjShowTest, &QPushButton::clicked, this, [this, showPatternPreview] {
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
        showPatternPreview(pat);
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
        if (zoomCapShow_) zoomCapShow_->clearImage();
    });

    // ---- Pro1/Pro2 投射按钮（条纹投射） ----
        connect(ui->capBtnPro1Trig, &QPushButton::clicked, this, [this, showPatternPreview,
            camSvc, camState, trigStatusLabel, capTopStatus, nowMs] {
        if (!projMgr_->isConnected(0)) {
            appendCapLog(QStringLiteral("错误: Pro1 未连接，请先扫描并连接设备"));
            return;
        }
        appendCapLog(QStringLiteral("Pro1 开始条纹投射..."));
        projMgr_->startPatternSequence(0, true);
        showPatternPreview(ui->capComboProjPattern->currentIndex());

        if (camState->connected.load() && !camState->fallbackSwitching.exchange(true)) {
            const int seq = camState->triggerSeq.fetch_add(1) + 1;
            const long long switchMs = nowMs();
            std::thread([this, camSvc, camState, trigStatusLabel, capTopStatus, seq, switchMs]() {
                QString err;
                camSvc->applyTrigger("On", "Line0", err);
                QMetaObject::invokeMethod(this, [this, trigStatusLabel]() {
                    ui->capComboTrigMode->setCurrentText("On");
                    ui->capComboTrigSrc->setCurrentText("Line0");
                    trigStatusLabel->setText(QStringLiteral("● 当前触发模式：硬触发/Line0（切换中）"));
                    trigStatusLabel->setStyleSheet("color:#D4A017;font-weight:700;");
                    appendCapLog(QStringLiteral("触发模式切换: Pro1触发 → 相机切换为Line0硬触发"));
                }, Qt::QueuedConnection);

                // 持续监听外部硬触发：当 0.3s 无新帧即回退软触发
                bool seenHardFrame = false;
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
                while (std::chrono::steady_clock::now() < deadline && camState->connected.load()) {
                    if (camState->triggerSeq.load() != seq) {
                        camState->fallbackSwitching.store(false);
                        return;
                    }
                    const long long last = camState->lastFrameMs.load();
                    const long long now = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    if (last > switchMs) {
                        seenHardFrame = true;
                        if ((now - last) > 300) {
                            break; // 0.3s 无硬触发帧
                        }
                    } else if ((now - switchMs) > 300) {
                        break; // 初始 0.3s 未收到硬触发
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(80));
                }

                camSvc->applyTrigger("On", "Software", err);
                QMetaObject::invokeMethod(this, [this, trigStatusLabel, capTopStatus, seenHardFrame, camState]() {
                    ui->capComboTrigMode->setCurrentText("On");
                    ui->capComboTrigSrc->setCurrentText("Software");
                    trigStatusLabel->setText(QStringLiteral("● 当前触发模式：软触发/Software"));
                    trigStatusLabel->setStyleSheet("color:#2F86F6;font-weight:600;");
                    appendCapLog(seenHardFrame
                        ? QStringLiteral("检测到投射结束（0.3s无硬触发帧），自动切换为软触发；投影仪投射保持正常")
                        : QStringLiteral("0.3s内未收到硬触发信号，自动切换为软触发；投影仪投射保持正常"));
                    capTopStatus->setText(QStringLiteral("相机分辨率：- | 帧率：0.00 fps | 累计帧数：%1 | 当前触发模式：On/Software")
                        .arg(camState->totalFrames.load()));
                }, Qt::QueuedConnection);
                camState->fallbackSwitching.store(false);
            }).detach();
        }
    });
    connect(ui->capBtnPro2Trig, &QPushButton::clicked, this, [this, showPatternPreview,
            camSvc, camState, trigStatusLabel, capTopStatus, nowMs] {
        if (!projMgr_->isConnected(1)) {
            appendCapLog(QStringLiteral("错误: Pro2 未连接，请先扫描并连接设备"));
            return;
        }
        appendCapLog(QStringLiteral("Pro2 开始条纹投射..."));
        projMgr_->startPatternSequence(1, true);
        showPatternPreview(ui->capComboProjPattern->currentIndex());

        if (camState->connected.load() && !camState->fallbackSwitching.exchange(true)) {
            const int seq = camState->triggerSeq.fetch_add(1) + 1;
            const long long switchMs = nowMs();
            std::thread([this, camSvc, camState, trigStatusLabel, capTopStatus, seq, switchMs]() {
                QString err;
                camSvc->applyTrigger("On", "Line0", err);
                QMetaObject::invokeMethod(this, [this, trigStatusLabel]() {
                    ui->capComboTrigMode->setCurrentText("On");
                    ui->capComboTrigSrc->setCurrentText("Line0");
                    trigStatusLabel->setText(QStringLiteral("● 当前触发模式：硬触发/Line0（切换中）"));
                    trigStatusLabel->setStyleSheet("color:#D4A017;font-weight:700;");
                    appendCapLog(QStringLiteral("触发模式切换: Pro2触发 → 相机切换为Line0硬触发"));
                }, Qt::QueuedConnection);

                bool seenHardFrame = false;
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
                while (std::chrono::steady_clock::now() < deadline && camState->connected.load()) {
                    if (camState->triggerSeq.load() != seq) {
                        camState->fallbackSwitching.store(false);
                        return;
                    }
                    const long long last = camState->lastFrameMs.load();
                    const long long now = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    if (last > switchMs) {
                        seenHardFrame = true;
                        if ((now - last) > 300) {
                            break;
                        }
                    } else if ((now - switchMs) > 300) {
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(80));
                }

                camSvc->applyTrigger("On", "Software", err);
                QMetaObject::invokeMethod(this, [this, trigStatusLabel, capTopStatus, seenHardFrame, camState]() {
                    ui->capComboTrigMode->setCurrentText("On");
                    ui->capComboTrigSrc->setCurrentText("Software");
                    trigStatusLabel->setText(QStringLiteral("● 当前触发模式：软触发/Software"));
                    trigStatusLabel->setStyleSheet("color:#2F86F6;font-weight:600;");
                    appendCapLog(seenHardFrame
                        ? QStringLiteral("检测到投射结束（0.3s无硬触发帧），自动切换为软触发；投影仪投射保持正常")
                        : QStringLiteral("0.3s内未收到硬触发信号，自动切换为软触发；投影仪投射保持正常"));
                    capTopStatus->setText(QStringLiteral("相机分辨率：- | 帧率：0.00 fps | 累计帧数：%1 | 当前触发模式：On/Software")
                        .arg(camState->totalFrames.load()));
                }, Qt::QueuedConnection);
                camState->fallbackSwitching.store(false);
            }).detach();
        }
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

    // ---- 自动采集非模态窗口 ----
    connect(autoCapBtn, &QPushButton::clicked, this, [this, autoUi, autoCapState, camState, appendAutoLog,
            makeNextNumericSessionDir, currentMs]() {
        if (!autoUi->dlg) {
            autoUi->dlg = new QDialog(this);
            autoUi->dlg->setWindowTitle(QStringLiteral("自动采集配置"));
            autoUi->dlg->setAttribute(Qt::WA_DeleteOnClose, false);
            autoUi->dlg->setModal(false);
            autoUi->dlg->resize(860, 680);
            autoUi->dlg->setStyleSheet(
                "QDialog{background:#FFFFFF;color:#333333;}"
                "QGroupBox{border:1px solid #E5E5E5;margin-top:8px;font-weight:600;background:#FFFFFF;}"
                "QGroupBox::title{subcontrol-origin: margin;left:8px;padding:0 4px;color:#333333;}"
                "QLineEdit,QComboBox,QSpinBox,QTextEdit{background:#FFFFFF;color:#333333;border:1px solid #E5E5E5;padding:3px;}"
                "QPushButton{background:#FFFFFF;color:#333333;border:1px solid #E5E5E5;padding:4px 10px;}"
                "QPushButton:hover{background:#F5F5F5;}"
                "QLabel{color:#333333;}"
            );

            auto* root = new QVBoxLayout(autoUi->dlg);

            auto* basicBox = new QGroupBox(QStringLiteral("基本参数"), autoUi->dlg);
            auto* basicLay = new QHBoxLayout(basicBox);
            auto* fpsSpin = new QSpinBox(basicBox); fpsSpin->setRange(1, 240); fpsSpin->setValue(20);
            auto* resCombo = new QComboBox(basicBox); resCombo->addItems({"1920×1080", "1280×720", "640×480"});
            auto* expSpin = new QSpinBox(basicBox); expSpin->setRange(10, 1000000); expSpin->setValue(30000);
            basicLay->addWidget(new QLabel(QStringLiteral("帧率:"), basicBox)); basicLay->addWidget(fpsSpin);
            basicLay->addWidget(new QLabel(QStringLiteral("分辨率:"), basicBox)); basicLay->addWidget(resCombo);
            basicLay->addWidget(new QLabel(QStringLiteral("曝光值:"), basicBox)); basicLay->addWidget(expSpin);

            auto* limitBox = new QGroupBox(QStringLiteral("录像/存图帧数或时长限制"), autoUi->dlg);
            auto* limitLay = new QHBoxLayout(limitBox);
            autoUi->maxFrameSpin = new QSpinBox(limitBox); autoUi->maxFrameSpin->setRange(0, 10000000); autoUi->maxFrameSpin->setValue(0);
            autoUi->maxTimeSpin = new QSpinBox(limitBox); autoUi->maxTimeSpin->setRange(0, 10000000); autoUi->maxTimeSpin->setValue(0);
            autoUi->limitFrameRadio = new QRadioButton(QStringLiteral("帧"), limitBox);
            autoUi->limitSecRadio = new QRadioButton(QStringLiteral("秒"), limitBox);
            autoUi->limitMinRadio = new QRadioButton(QStringLiteral("分钟"), limitBox);
            autoUi->limitFrameRadio->setChecked(true);
            limitLay->addWidget(new QLabel(QStringLiteral("最大帧数:"), limitBox)); limitLay->addWidget(autoUi->maxFrameSpin);
            limitLay->addSpacing(16);
            limitLay->addWidget(new QLabel(QStringLiteral("最大时长:"), limitBox)); limitLay->addWidget(autoUi->maxTimeSpin);
            limitLay->addSpacing(10);
            limitLay->addWidget(autoUi->limitFrameRadio);
            limitLay->addWidget(autoUi->limitSecRadio);
            limitLay->addWidget(autoUi->limitMinRadio);

            auto* saveBox = new QGroupBox(QStringLiteral("保存文件设置"), autoUi->dlg);
            auto* saveLay = new QVBoxLayout(saveBox);

            auto* pathRow = new QHBoxLayout();
            autoUi->pro1Edit = new QLineEdit(saveBox);
            autoUi->pro2Edit = new QLineEdit(saveBox);
            autoUi->pro1Edit->setPlaceholderText(QStringLiteral("Pro1文件保存路径"));
            autoUi->pro2Edit->setPlaceholderText(QStringLiteral("Pro2文件保存路径"));
            auto* pro1Browse = new QPushButton(QStringLiteral("浏览"), saveBox);
            auto* pro2Browse = new QPushButton(QStringLiteral("浏览"), saveBox);
            pathRow->addWidget(new QLabel(QStringLiteral("Pro1文件保存路径:"), saveBox));
            pathRow->addWidget(autoUi->pro1Edit); pathRow->addWidget(pro1Browse);
            pathRow->addSpacing(10);
            pathRow->addWidget(new QLabel(QStringLiteral("Pro2文件保存路径:"), saveBox));
            pathRow->addWidget(autoUi->pro2Edit); pathRow->addWidget(pro2Browse);
            saveLay->addLayout(pathRow);

            auto* imgRow = new QHBoxLayout();
            autoUi->saveEnableCheck = new QCheckBox(QStringLiteral("存图"), saveBox);
            autoUi->saveEnableCheck->setChecked(true);
            autoUi->fmtCombo = new QComboBox(saveBox); autoUi->fmtCombo->addItems({"BMP", "PNG", "JPG"});
            autoUi->byFrameRadio = new QRadioButton(QStringLiteral("按帧数"), saveBox); autoUi->byFrameRadio->setChecked(true);
            autoUi->byTimeRadio = new QRadioButton(QStringLiteral("按时间"), saveBox);
            autoUi->intervalSpin = new QSpinBox(saveBox); autoUi->intervalSpin->setRange(1, 1000000); autoUi->intervalSpin->setValue(1);
            imgRow->addWidget(autoUi->saveEnableCheck);
            imgRow->addSpacing(8);
            imgRow->addWidget(new QLabel(QStringLiteral("图片格式:"), saveBox));
            imgRow->addWidget(autoUi->fmtCombo);
            imgRow->addSpacing(20);
            imgRow->addWidget(autoUi->byFrameRadio);
            imgRow->addWidget(autoUi->byTimeRadio);
            imgRow->addWidget(new QLabel(QStringLiteral("间隔值:"), saveBox));
            imgRow->addWidget(autoUi->intervalSpin);
            saveLay->addLayout(imgRow);

            auto* trigRow = new QHBoxLayout();
            auto* actPro1Btn = new QPushButton(QStringLiteral("Pro1触发"), saveBox);
            auto* actPro2Btn = new QPushButton(QStringLiteral("Pro2触发"), saveBox);
            trigRow->addWidget(actPro1Btn);
            trigRow->addWidget(actPro2Btn);
            trigRow->addStretch(1);
            saveLay->addLayout(trigRow);

            auto* infoBox = new QGroupBox(QStringLiteral("信息"), autoUi->dlg);
            auto* infoLay = new QVBoxLayout(infoBox);
            autoUi->infoBuffer = new QLabel(QStringLiteral("Buffer占用比：0%"), infoBox);
            autoUi->infoDisk = new QLabel(QStringLiteral("硬盘剩余帧数：-"), infoBox);
            autoUi->infoTotal = new QLabel(QStringLiteral("已采集帧数：0"), infoBox);
            infoLay->addWidget(autoUi->infoBuffer);
            infoLay->addWidget(autoUi->infoDisk);
            infoLay->addWidget(autoUi->infoTotal);

            auto* logBox = new QGroupBox(QStringLiteral("信息日志"), autoUi->dlg);
            auto* logLay = new QVBoxLayout(logBox);
            autoUi->log = new QTextEdit(logBox);
            autoUi->log->setReadOnly(true);
            autoUi->log->setMinimumHeight(170);
            auto* clearLogBtn = new QPushButton(QStringLiteral("清空日志"), logBox);
            logLay->addWidget(autoUi->log);
            logLay->addWidget(clearLogBtn, 0, Qt::AlignRight);

            auto* buttons = new QDialogButtonBox(Qt::Horizontal, autoUi->dlg);
            auto* okBtn = buttons->addButton(QStringLiteral("确认"), QDialogButtonBox::AcceptRole);
            auto* cancelBtn = buttons->addButton(QStringLiteral("取消"), QDialogButtonBox::RejectRole);
            auto* applyBtn = buttons->addButton(QStringLiteral("应用"), QDialogButtonBox::ApplyRole);
            Q_UNUSED(okBtn);
            Q_UNUSED(cancelBtn);
            Q_UNUSED(applyBtn);

            root->addWidget(basicBox);
            root->addWidget(limitBox);
            root->addWidget(saveBox);
            root->addWidget(infoBox);
            root->addWidget(logBox, 1);
            root->addWidget(buttons, 0, Qt::AlignHCenter);

            connect(pro1Browse, &QPushButton::clicked, autoUi->dlg, [this, autoUi]() {
                const auto dir = QFileDialog::getExistingDirectory(this, QStringLiteral("选择Pro1保存路径"));
                if (!dir.isEmpty() && autoUi->pro1Edit) autoUi->pro1Edit->setText(dir);
            });
            connect(pro2Browse, &QPushButton::clicked, autoUi->dlg, [this, autoUi]() {
                const auto dir = QFileDialog::getExistingDirectory(this, QStringLiteral("选择Pro2保存路径"));
                if (!dir.isEmpty() && autoUi->pro2Edit) autoUi->pro2Edit->setText(dir);
            });

            auto applyDialogState = [autoCapState, autoUi]() {
                std::lock_guard<std::mutex> lock(autoCapState->mtx);
                if (autoUi->pro1Edit) autoCapState->pro1Base = autoUi->pro1Edit->text().trimmed();
                if (autoUi->pro2Edit) autoCapState->pro2Base = autoUi->pro2Edit->text().trimmed();
                if (autoUi->fmtCombo) autoCapState->imageFormat = autoUi->fmtCombo->currentText().trimmed();
                if (autoUi->saveEnableCheck) autoCapState->enableSave = autoUi->saveEnableCheck->isChecked();
                if (autoUi->byFrameRadio && autoUi->byFrameRadio->isChecked()) autoCapState->intervalMode = 0;
                else if (autoUi->byTimeRadio && autoUi->byTimeRadio->isChecked()) autoCapState->intervalMode = 1;
                if (autoUi->intervalSpin) autoCapState->intervalValue = std::max(1, autoUi->intervalSpin->value());
                if (autoUi->limitFrameRadio && autoUi->limitFrameRadio->isChecked()) autoCapState->limitMode = 0;
                else if (autoUi->limitSecRadio && autoUi->limitSecRadio->isChecked()) autoCapState->limitMode = 1;
                else if (autoUi->limitMinRadio && autoUi->limitMinRadio->isChecked()) autoCapState->limitMode = 2;
                if (autoUi->maxFrameSpin) autoCapState->maxFrameLimit = autoUi->maxFrameSpin->value();
                if (autoUi->maxTimeSpin) autoCapState->maxTimeLimit = autoUi->maxTimeSpin->value();
            };

            connect(buttons, &QDialogButtonBox::accepted, autoUi->dlg, [applyDialogState, autoUi]() {
                applyDialogState();
                if (autoUi->dlg) autoUi->dlg->hide();
            });
            connect(buttons, &QDialogButtonBox::rejected, autoUi->dlg, [autoUi]() {
                if (autoUi->dlg) autoUi->dlg->hide();
            });
            connect(applyBtn, &QPushButton::clicked, autoUi->dlg, [applyDialogState, appendAutoLog]() {
                applyDialogState();
                appendAutoLog(QStringLiteral("自动采集参数已应用"));
            });

            connect(clearLogBtn, &QPushButton::clicked, autoUi->dlg, [autoUi]() {
                if (autoUi->log) autoUi->log->clear();
            });

            connect(actPro1Btn, &QPushButton::clicked, autoUi->dlg, [autoCapState, autoUi, appendAutoLog, makeNextNumericSessionDir, currentMs]() {
                if (!autoUi->pro1Edit || autoUi->pro1Edit->text().trimmed().isEmpty()) {
                    appendAutoLog(QStringLiteral("Pro1路径为空，请先选择路径"));
                    return;
                }
                std::lock_guard<std::mutex> lock(autoCapState->mtx);
                autoCapState->activeProjector = 1;
                autoCapState->pro1Base = autoUi->pro1Edit->text().trimmed();
                autoCapState->activeSessionDir = makeNextNumericSessionDir(autoCapState->pro1Base);
                autoCapState->nextImageIndex = 1;
                autoCapState->savedCount = 0;
                autoCapState->sampledFrames = 0;
                autoCapState->sessionStartMs = currentMs();
                autoCapState->lastSavedMs = 0;
                autoCapState->lastSavedFrameTotal = 0;
                appendAutoLog(QStringLiteral("Pro1触发激活，当前会话目录: %1").arg(autoCapState->activeSessionDir));
            });
            connect(actPro2Btn, &QPushButton::clicked, autoUi->dlg, [autoCapState, autoUi, appendAutoLog, makeNextNumericSessionDir, currentMs]() {
                if (!autoUi->pro2Edit || autoUi->pro2Edit->text().trimmed().isEmpty()) {
                    appendAutoLog(QStringLiteral("Pro2路径为空，请先选择路径"));
                    return;
                }
                std::lock_guard<std::mutex> lock(autoCapState->mtx);
                autoCapState->activeProjector = 2;
                autoCapState->pro2Base = autoUi->pro2Edit->text().trimmed();
                autoCapState->activeSessionDir = makeNextNumericSessionDir(autoCapState->pro2Base);
                autoCapState->nextImageIndex = 1;
                autoCapState->savedCount = 0;
                autoCapState->sampledFrames = 0;
                autoCapState->sessionStartMs = currentMs();
                autoCapState->lastSavedMs = 0;
                autoCapState->lastSavedFrameTotal = 0;
                appendAutoLog(QStringLiteral("Pro2触发激活，当前会话目录: %1").arg(autoCapState->activeSessionDir));
            });

            auto* infoTimer = new QTimer(autoUi->dlg);
            infoTimer->setInterval(400);
            connect(infoTimer, &QTimer::timeout, autoUi->dlg, [autoCapState, camState, autoUi]() {
                if (!autoUi->infoBuffer || !autoUi->infoDisk || !autoUi->infoTotal) return;
                long long saved = 0;
                {
                    std::lock_guard<std::mutex> lock(autoCapState->mtx);
                    saved = autoCapState->savedCount;
                }
                const long long total = camState->totalFrames.load();
                const long long bufferPct = (total <= 0) ? 0 : std::min<long long>(100, (saved * 100) / std::max<long long>(1, total));
                autoUi->infoBuffer->setText(QStringLiteral("Buffer占用比：%1%").arg(bufferPct));

                QStorageInfo si(QDir::currentPath());
                const qint64 bytes = si.bytesAvailable();
                const qint64 estFrameBytes = 1024 * 1024;
                const qint64 remainFrames = estFrameBytes > 0 ? bytes / estFrameBytes : 0;
                autoUi->infoDisk->setText(QStringLiteral("硬盘剩余帧数：%1").arg(remainFrames));
                autoUi->infoTotal->setText(QStringLiteral("已采集帧数：%1").arg(total));
            });
            infoTimer->start();
        }

        if (autoUi->dlg) {
            autoUi->dlg->show();
            autoUi->dlg->raise();
            autoUi->dlg->activateWindow();
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

void MainWindow::onResBtnScanPro1BrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择Pro1扫描图像文件夹 (如 BGA静态/1/1)"));
    if (!dir.isEmpty()) ui->resEditScanPro1->setText(dir);
}

void MainWindow::onResBtnScanPro2BrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择Pro2扫描图像文件夹"));
    if (!dir.isEmpty()) ui->resEditScanPro2->setText(dir);
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

void MainWindow::onResBtnMultiRootBrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择多次重建根目录 (包含 1/ 和 2/ 子目录)"));
    if (!dir.isEmpty()) ui->resEditMultiRoot->setText(dir);
}

void MainWindow::onResBtnMultiSavePathBrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择多次重建保存目录"));
    if (!dir.isEmpty()) ui->resEditMultiSavePath->setText(dir);
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
    if (state_ != State::READY && state_ != State::DONE) {
        appendLog(QStringLiteral("请先加载标定文件")); return;
    }
    int mode = ui->resComboSaveMode->currentIndex();
    if (mode == 0) {
        startSingleReconstruction();
    } else {
        startMultiReconstruction();
    }
}

void MainWindow::startSingleReconstruction() {
    appendLog(QStringLiteral("开始单次重建..."));
    setState(State::RECONSTRUCTING);
    emit progressUpdated(0);

    QString scanPro1Dir = ui->resEditScanPro1->text().trimmed();
    QString scanPro2Dir = ui->resEditScanPro2->text().trimmed();
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
    scanPro1Dir = resolveScanDir(scanPro1Dir, QStringLiteral("data/pic/BGA\u9759\u6001/1/1"));
    scanPro2Dir = resolveScanDir(scanPro2Dir, QStringLiteral("data/pic/BGA\u9759\u6001/2/1"));
    appendLog(QStringLiteral("Pro1\u8def\u5f84: ") + scanPro1Dir);
    appendLog(QStringLiteral("Pro2\u8def\u5f84: ") + scanPro2Dir);

    auto params = reconParams_;
    auto calib = dualCalib_;

    std::thread([this, pro1Dir = scanPro1Dir.toStdString(),
                 pro2Dir = scanPro2Dir.toStdString(), params, calib]() {
        try {
            emit progressUpdated(5);
            emit logMessage(QStringLiteral("\u8bfb\u53d6\u56fe\u50cf..."));

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

            auto fut1 = std::async(std::launch::async, loadImgs, pro1Dir, 1, 24);
            auto fut2 = std::async(std::launch::async, loadImgs, pro2Dir, 1, 24);
            auto imgs1 = fut1.get();
            emit progressUpdated(15);
            auto imgs2 = fut2.get();
            emit progressUpdated(25);

            cv::Mat colorImg = imreadSafe(pro1Dir + "/49.bmp", cv::IMREAD_GRAYSCALE);
            if (colorImg.empty())
                colorImg = imreadSafe(pro2Dir + "/49.bmp", cv::IMREAD_GRAYSCALE);
            if (colorImg.empty() && !imgs1.empty()) {
                colorImg = cv::Mat(imgs1[0].rows, imgs1[0].cols, CV_8U, cv::Scalar(128));
                emit logMessage(QStringLiteral("\u8b66\u544a\uff1a\u7b2c49\u5e27\u53c2\u8003\u56fe\u4e0d\u5b58\u5728\uff0c\u4f7f\u7528\u7eaf\u7070\u66ff\u4ee3"));
            }

            emit logMessage(QStringLiteral("\u6267\u884c\u91cd\u5efa\u6d41\u7a0b..."));
            emit progressUpdated(30);

            tp::ReconstructionPipeline pipeline;
            auto result = pipeline.runDual(imgs1, imgs2, colorImg, calib, params);
            emit progressUpdated(90);

            if (!result.success) {
                emit logMessage(QString("\u91cd\u5efa\u5931\u8d25: ") +
                    QString::fromStdString(result.errorMsg));
                QMetaObject::invokeMethod(this, [this]() {
                    setState(State::READY);
                }, Qt::QueuedConnection);
                return;
            }

            auto colorCloud = result.cloud;
            emit progressUpdated(100);
            emit logMessage(QStringLiteral("\u91cd\u5efa\u5b8c\u6210: %1 \u70b9").arg(colorCloud->size()));

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
            emit logMessage(QStringLiteral("相位图: Pro1=%1x%2 Pro2=%3x%4")
                .arg(result.absPhase1.cols).arg(result.absPhase1.rows)
                .arg(result.absPhase2.cols).arg(result.absPhase2.rows));

            QMetaObject::invokeMethod(this, [this, colorCloud, img1, img2]() {
                currentCloud_ = colorCloud;
                // Add to multi-cloud system
                std::string id = generateCloudId();
                tp::CloudEntry entry;
                entry.id = id;
                entry.name = "Recon_" + std::to_string(cloudIdCounter_);
                entry.cloud = colorCloud;
                entry.visible = true;
                cloudStore_[id] = entry;
                vtkWidget_->addOrUpdateCloud(id, colorCloud, false);

                auto* item = new QListWidgetItem(
                    QStringLiteral("重建点云 (%1 点)").arg(colorCloud->size()));
                item->setData(Qt::UserRole, QString::fromStdString(id));
                item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
                item->setCheckState(Qt::Checked);
                ui->pcCloudListWidget->addItem(item);
                ui->pcCloudListWidget->setCurrentItem(item);
                if (!img1.isNull())
                    zoomResPhase1_->setImage(img1);
                if (!img2.isNull())
                    zoomResPhase2_->setImage(img2);
                setState(State::DONE);
            }, Qt::QueuedConnection);

        } catch (const std::exception& e) {
            emit logMessage(QString("\u91cd\u5efa\u9519\u8bef: ") + e.what());
            QMetaObject::invokeMethod(this, [this]() {
                setState(State::READY);
            }, Qt::QueuedConnection);
        }
    }).detach();
}

void MainWindow::startMultiReconstruction() {
    appendLog(QStringLiteral("开始多次重建..."));
    setState(State::RECONSTRUCTING);
    emit progressUpdated(0);

    QString rootDir = ui->resEditMultiRoot->text().trimmed();
    QString saveDir = ui->resEditMultiSavePath->text().trimmed();

    auto resolveDir = [](const QString& input, const QString& defaultRel) -> QString {
        if (!input.isEmpty() && QDir(input).exists())
            return QDir(input).absolutePath();
        if (QDir(QDir::currentPath()).exists(defaultRel))
            return QDir(QDir::currentPath()).absoluteFilePath(defaultRel);
        QDir exeDir(QCoreApplication::applicationDirPath());
        if (exeDir.cdUp() && exeDir.cdUp() && exeDir.exists(defaultRel))
            return exeDir.absoluteFilePath(defaultRel);
        return input;
    };
    rootDir = resolveDir(rootDir, QStringLiteral("data/pic/BGA\u9759\u6001"));

    if (saveDir.isEmpty()) {
        appendLog(QStringLiteral("请设置多次重建保存路径"));
        setState(State::READY);
        return;
    }

    auto params = reconParams_;
    auto calib = dualCalib_;
    int maxCount = ui->resSpinRepeatCount->value();

    std::thread([this, root = rootDir.toStdString(),
                 save = saveDir.toStdString(), params, calib, maxCount]() {
        try {
            namespace fs = std::filesystem;
            fs::path pro1Root = fs::u8path(root) / "1";
            fs::path pro2Root = fs::u8path(root) / "2";

            std::vector<int> iterations;
            for (auto& entry : fs::directory_iterator(pro1Root)) {
                if (entry.is_directory()) {
                    auto name = entry.path().filename().string();
                    try { iterations.push_back(std::stoi(name)); }
                    catch (...) {}
                }
            }
            std::sort(iterations.begin(), iterations.end());
            // 根据用户设置的重建次数截断
            if (static_cast<int>(iterations.size()) > maxCount)
                iterations.resize(maxCount);
            int total = static_cast<int>(iterations.size());

            emit logMessage(QStringLiteral("\u53d1\u73b0 %1 \u4e2a\u6d4b\u91cf\u5b50\u6587\u4ef6\u5939").arg(total));
            fs::create_directories(fs::u8path(save));

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

            for (int idx = 0; idx < total; ++idx) {
                int iter = iterations[idx];
                std::string pro1Dir = (pro1Root / std::to_string(iter)).string();
                std::string pro2Dir = (pro2Root / std::to_string(iter)).string();

                int overallPercent = (idx * 100) / total;
                emit progressUpdated(overallPercent);
                emit logMessage(QStringLiteral("\u91cd\u5efa\u7b2c %1/%2 \u6b21 (\u6587\u4ef6\u5939: %3)")
                    .arg(idx + 1).arg(total).arg(iter));

                auto f1 = std::async(std::launch::async, loadImgs, pro1Dir, 1, 24);
                auto f2 = std::async(std::launch::async, loadImgs, pro2Dir, 1, 24);
                auto imgs1 = f1.get();
                auto imgs2 = f2.get();

                cv::Mat colorImg = imreadSafe(pro1Dir + "/49.bmp", cv::IMREAD_GRAYSCALE);
                if (colorImg.empty() && !imgs1.empty())
                    colorImg = cv::Mat(imgs1[0].rows, imgs1[0].cols, CV_8U, cv::Scalar(128));

                tp::ReconstructionPipeline pipeline;
                auto result = pipeline.runDual(imgs1, imgs2, colorImg, calib, params);

                if (!result.success) {
                    emit logMessage(QStringLiteral("\u7b2c %1 \u6b21\u91cd\u5efa\u5931\u8d25: %2")
                        .arg(iter).arg(QString::fromStdString(result.errorMsg)));
                    continue;
                }

                auto savePath = fs::u8path(save) / (std::to_string(iter) + ".ply");
                tp::PointCloudIO::savePLY(result.cloud, savePath.string());
                emit logMessage(QStringLiteral("\u7b2c %1 \u6b21\u70b9\u4e91\u5df2\u4fdd\u5b58: %2")
                    .arg(iter).arg(QString::fromStdString(savePath.string())));
            }

            emit progressUpdated(100);
            emit logMessage(QStringLiteral("\u591a\u6b21\u91cd\u5efa\u5b8c\u6210: \u5171 %1 \u6b21").arg(total));

            QMetaObject::invokeMethod(this, [this]() {
                setState(State::DONE);
            }, Qt::QueuedConnection);

        } catch (const std::exception& e) {
            emit logMessage(QString("\u591a\u6b21\u91cd\u5efa\u9519\u8bef: ") + e.what());
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
    bool isMulti = (index == 1);
    ui->resSpinRepeatCount->setEnabled(isMulti);
    ui->resEditMultiRoot->setVisible(isMulti);
    ui->resBtnMultiRootBrowse->setVisible(isMulti);
    ui->resLblMultiRoot->setVisible(isMulti);
    ui->resEditMultiSavePath->setVisible(isMulti);
    ui->resBtnMultiSavePathBrowse->setVisible(isMulti);
    ui->resLblMultiSavePath->setVisible(isMulti);
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
    auto* entry = selectedEntry();
    if (!entry || !entry->cloud || entry->cloud->empty()) {
        // Fallback to legacy currentCloud_
        if (!currentCloud_ || currentCloud_->empty()) return;
        for (auto& pt : currentCloud_->points) pt.z = -pt.z;
        vtkWidget_->updateCloud(currentCloud_);
        return;
    }
    for (auto& pt : entry->cloud->points) pt.z = -pt.z;
    currentCloud_ = entry->cloud;
    vtkWidget_->addOrUpdateCloud(entry->id, entry->cloud, entry->heightColored);
}

// ============================================================
// 点云处理按钮
// ============================================================

void MainWindow::onPcBtnImportCloudClicked() {
    auto paths = QFileDialog::getOpenFileNames(this,
        QStringLiteral("导入点云"), {},
        "Point Cloud (*.ply *.pcd)");
    if (paths.isEmpty()) return;
    for (const auto& path : paths) {
        try {
            auto cloud = tp::PointCloudIO::load(path.toStdString());
            auto rgbCloud = xyzToRgb(cloud);
            QFileInfo fi(path);

            std::string id = generateCloudId();
            tp::CloudEntry entry;
            entry.id = id;
            entry.name = fi.fileName().toStdString();
            entry.filePath = path.toStdString();
            entry.cloud = rgbCloud;
            entry.visible = true;
            entry.heightColored = false;
            cloudStore_[id] = entry;

            currentCloud_ = rgbCloud;
            vtkWidget_->addOrUpdateCloud(id, rgbCloud, false);

            auto* item = new QListWidgetItem(fi.fileName());
            item->setData(Qt::UserRole, QString::fromStdString(id));
            item->setToolTip(path);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(Qt::Checked);
            ui->pcCloudListWidget->addItem(item);
            ui->pcCloudListWidget->setCurrentItem(item);

            appendLog(QStringLiteral("导入点云: ") + fi.fileName() +
                QStringLiteral(" (%1 点)").arg(cloud->size()));
        } catch (const std::exception& e) {
            appendLog(QStringLiteral("导入失败 [%1]: %2")
                .arg(QFileInfo(path).fileName())
                .arg(e.what()));
        }
    }
}

void MainWindow::onPcCloudListContextMenu(const QPoint& pos) {
    auto* item = ui->pcCloudListWidget->itemAt(pos);
    QMenu menu(this);
    auto* actDelete = menu.addAction(QStringLiteral("删除"));
    auto* actDeleteAll = menu.addAction(QStringLiteral("删除全部"));
    menu.addSeparator();
    auto* actReload = menu.addAction(QStringLiteral("重新加载"));
    auto* actShowInVtk = menu.addAction(QStringLiteral("在3D视图中显示"));

    // 仅当选中有效项时启用
    actDelete->setEnabled(item != nullptr);
    actReload->setEnabled(item != nullptr);
    actShowInVtk->setEnabled(item != nullptr);

    auto* chosen = menu.exec(ui->pcCloudListWidget->mapToGlobal(pos));
    if (!chosen) return;

    if (chosen == actDelete && item) {
        std::string id = item->data(Qt::UserRole).toString().toStdString();
        cloudStore_.erase(id);
        vtkWidget_->removeCloud(id);
        if (selectedCloudId_ == id) { selectedCloudId_.clear(); currentCloud_.reset(); }
        delete ui->pcCloudListWidget->takeItem(
            ui->pcCloudListWidget->row(item));
        appendLog(QStringLiteral("已删除点云"));
    } else if (chosen == actDeleteAll) {
        ui->pcCloudListWidget->clear();
        cloudStore_.clear();
        selectedCloudId_.clear();
        vtkWidget_->removeAllClouds();
        currentCloud_.reset();
        appendLog(QStringLiteral("已删除全部点云"));
    } else if (chosen == actReload && item) {
        std::string id = item->data(Qt::UserRole).toString().toStdString();
        auto it = cloudStore_.find(id);
        if (it == cloudStore_.end() || it->second.filePath.empty()) return;
        try {
            auto cloud = xyzToRgb(tp::PointCloudIO::load(it->second.filePath));
            it->second.cloud = cloud;
            currentCloud_ = cloud;
            vtkWidget_->addOrUpdateCloud(id, cloud, it->second.heightColored);
            appendLog(QStringLiteral("重新加载: ") +
                QString::fromStdString(it->second.name));
        } catch (const std::exception& e) {
            appendLog(QStringLiteral("重新加载失败: ") + e.what());
        }
    } else if (chosen == actShowInVtk && item) {
        QString cloudId = item->data(Qt::UserRole).toString();
        std::string id = cloudId.toStdString();
        auto it = cloudStore_.find(id);
        if (it != cloudStore_.end()) {
            currentCloud_ = it->second.cloud;
            vtkWidget_->showBoundingBox(id);
        }
    }
}

void MainWindow::onPcCloudListItemChanged(QListWidgetItem* item) {
    if (!item) return;
    std::string id = item->data(Qt::UserRole).toString().toStdString();
    bool visible = (item->checkState() == Qt::Checked);
    auto it = cloudStore_.find(id);
    if (it != cloudStore_.end()) {
        it->second.visible = visible;
        vtkWidget_->setCloudVisible(id, visible);
    }
}

void MainWindow::onPcCloudListCurrentChanged(QListWidgetItem* current, QListWidgetItem*) {
    if (!current) {
        selectedCloudId_.clear();
        vtkWidget_->hideBoundingBox();
        return;
    }
    std::string id = current->data(Qt::UserRole).toString().toStdString();
    auto it = cloudStore_.find(id);
    if (it == cloudStore_.end()) return;

    // Only select checked items
    if (!it->second.visible) {
        vtkWidget_->hideBoundingBox();
        return;
    }

    selectedCloudId_ = id;
    currentCloud_ = it->second.cloud;
    vtkWidget_->showBoundingBox(id);

    // Sync height color checkbox
    ui->pcCheckHeightColor->blockSignals(true);
    ui->pcCheckHeightColor->setChecked(it->second.heightColored);
    ui->pcCheckHeightColor->blockSignals(false);
}

void MainWindow::onPcCheckHeightColorChanged(int state) {
    auto* entry = selectedEntry();
    if (!entry || !entry->cloud) return;
    bool heightColored = (state == Qt::Checked);
    entry->heightColored = heightColored;
    vtkWidget_->setCloudHeightColored(entry->id, entry->cloud, heightColored);
}

void MainWindow::onPcBtnCropROIClicked() {
    if (!vtkWidget_) return;
    if (vtkWidget_->isROICropEnabled()) {
        vtkWidget_->enableROICrop(false);
        appendLog(QStringLiteral("ROI 裁剪模式已关闭"));
    } else {
        if (cloudStore_.empty() && !currentCloud_) {
            appendLog(QStringLiteral("请先导入或重建点云"));
            return;
        }
        vtkWidget_->enableROICrop(true, false);
        appendLog(QStringLiteral("ROI 裁剪模式已开启 (WASD平移/QE升降/滚轮缩放/R重置/1AABB/2OBB/C裁剪)"));
    }
}

void MainWindow::performROICrop() {
    auto* entry = selectedEntry();
    if (!entry || !entry->cloud || entry->cloud->empty()) {
        appendLog(QStringLiteral("请先选中点云"));
        return;
    }

    double bounds[6];
    vtkWidget_->getROITransform(bounds);

    auto src = entry->cloud;
    auto cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (const auto& pt : src->points) {
        if (pt.x >= bounds[0] && pt.x <= bounds[1] &&
            pt.y >= bounds[2] && pt.y <= bounds[3] &&
            pt.z >= bounds[4] && pt.z <= bounds[5]) {
            cropped->push_back(pt);
        }
    }

    if (cropped->empty()) {
        appendLog(QStringLiteral("裁剪结果为空"));
        return;
    }

    // Color cropped points green
    for (auto& pt : cropped->points) {
        pt.r = 100; pt.g = 255; pt.b = 100;
    }

    std::string id = generateCloudId();
    tp::CloudEntry newEntry;
    newEntry.id = id;
    newEntry.name = "ROI_crop_" + std::to_string(cloudIdCounter_);
    newEntry.cloud = cropped;
    newEntry.visible = true;
    cloudStore_[id] = newEntry;

    currentCloud_ = cropped;
    vtkWidget_->addOrUpdateCloud(id, cropped, false);

    auto* item = new QListWidgetItem(QString::fromStdString(newEntry.name));
    item->setData(Qt::UserRole, QString::fromStdString(id));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Checked);
    ui->pcCloudListWidget->addItem(item);
    ui->pcCloudListWidget->setCurrentItem(item);

    vtkWidget_->enableROICrop(false);
    appendLog(QStringLiteral("ROI 裁剪完成: %1 点").arg(cropped->size()));
}
void MainWindow::onPcBtnCropPlaneClicked() {
    appendLog(QStringLiteral("平面裁剪功能开发中"));
}
void MainWindow::onPcBtnSubsampleClicked() {
    auto* entry = selectedEntry();
    auto cloud = entry ? entry->cloud : currentCloud_;
    if (!cloud || cloud->empty()) {
        appendLog(QStringLiteral("请先导入或重建点云")); return;
    }
    float leafSize = static_cast<float>(ui->pcSpinVoxelSize->value());
    std::string entryId = entry ? entry->id : "";
    std::thread([this, cloud, leafSize, entryId]() {
        try {
            tp::PointCloudFilter f;
            auto xyzIn = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            for (const auto& p : cloud->points)
                xyzIn->push_back({p.x, p.y, p.z});
            auto filtered = f.voxelGrid(xyzIn, leafSize);
            auto rgbOut = xyzToRgb(filtered);
            QMetaObject::invokeMethod(this, [this, rgbOut, entryId]() {
                currentCloud_ = rgbOut;
                if (!entryId.empty()) {
                    auto it = cloudStore_.find(entryId);
                    if (it != cloudStore_.end()) {
                        it->second.cloud = rgbOut;
                        vtkWidget_->addOrUpdateCloud(entryId, rgbOut,
                            it->second.heightColored);
                    }
                } else {
                    vtkWidget_->updateCloud(rgbOut);
                }
                appendLog(QStringLiteral("体素降采样完成: %1 点").arg(rgbOut->size()));
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
    auto* entry = selectedEntry();
    auto cloud = entry ? entry->cloud : currentCloud_;
    if (!cloud || cloud->empty()) {
        appendLog(QStringLiteral("请先导入或重建点云")); return;
    }
    int meanK = ui->pcSpinSORK->value();
    double stdDev = ui->pcSpinSORStd->value();
    std::string entryId = entry ? entry->id : "";
    std::thread([this, cloud, meanK, stdDev, entryId]() {
        try {
            tp::PointCloudFilter f;
            auto xyzIn = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            for (const auto& p : cloud->points)
                xyzIn->push_back({p.x, p.y, p.z});
            auto filtered = f.SOR(xyzIn, meanK, stdDev);
            auto rgbOut = xyzToRgb(filtered);
            QMetaObject::invokeMethod(this, [this, rgbOut, entryId]() {
                currentCloud_ = rgbOut;
                if (!entryId.empty()) {
                    auto it = cloudStore_.find(entryId);
                    if (it != cloudStore_.end()) {
                        it->second.cloud = rgbOut;
                        vtkWidget_->addOrUpdateCloud(entryId, rgbOut,
                            it->second.heightColored);
                    }
                } else {
                    vtkWidget_->updateCloud(rgbOut);
                }
                appendLog(QStringLiteral("SOR完成: %1 点").arg(rgbOut->size()));
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
            auto xyzIn = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            for (const auto& p : cloud->points)
                xyzIn->push_back({p.x, p.y, p.z});
            auto filtered = f.passThrough(xyzIn, axis, -999.0f, 999.0f);
            auto rgbOut = xyzToRgb(filtered);
            QMetaObject::invokeMethod(this, [this, rgbOut]() {
                currentCloud_ = rgbOut;
                vtkWidget_->updateCloud(rgbOut);
                appendLog(QStringLiteral("直通滤波完成: %1 点").arg(rgbOut->size()));
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
    auto* entry = selectedEntry();
    auto cloud = entry ? entry->cloud : currentCloud_;
    if (!cloud || cloud->empty()) { appendLog(QStringLiteral("没有点云")); return; }
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

void MainWindow::updateBgaChartsFromCurrentResults() {
    if (bgaCoplanarities_.empty()) {
        appendBgaLog(QStringLiteral("暂无共面度数据，无法绘制图表"));
        return;
    }

    // 直接使用 picsrc 同源算法: 柱状图=焊球偏差分布，折线图=重复测量共面度
    QImage barQImg = renderPicSrcBgaDeviationBarChart(lastBgaResults_);
    QImage lineQImg = renderPicSrcBgaLineChart(bgaCoplanarities_);

    if (!barQImg.isNull() && zoomBgaBar_) {
        zoomBgaBar_->setImage(barQImg);
        appendBgaLog(QStringLiteral("BGA图像更新: 偏差柱状图 %1x%2")
            .arg(barQImg.width()).arg(barQImg.height()));
        appendBgaLog(QStringLiteral("柱状图来源: output/bga_ball_deviation.png"));
    } else {
        appendBgaLog(QStringLiteral("BGA图像更新失败: 偏差柱状图生成失败或控件为空"));
    }

    if (!lineQImg.isNull() && zoomBgaLine_) {
        zoomBgaLine_->setImage(lineQImg);
        appendBgaLog(QStringLiteral("BGA图像更新: 折线图 %1x%2")
            .arg(lineQImg.width()).arg(lineQImg.height()));
        appendBgaLog(QStringLiteral("折线图来源: output/bga_line_chart.png"));
    } else {
        appendBgaLog(QStringLiteral("BGA图像更新失败: 折线图生成失败或控件为空"));
    }
}

void MainWindow::onBgaBtnStartClicked() {
    QString imgDir = ui->bgaEditImgFolder->text().trimmed();
    QString cloudDir = ui->bgaEditCloudFolder->text().trimmed();
    int count = ui->bgaSpinCount->value();

    if (imgDir.isEmpty() || cloudDir.isEmpty()) {
        appendBgaLog(QStringLiteral("请先设置图像文件夹和点云文件夹"));
        return;
    }

    appendBgaLog(QStringLiteral("开始测量 (%1 次)...").arg(count));
    ui->bgaBtnStart->setEnabled(false);

    auto calib = dualCalib_;

    std::thread([this, imgFolder = imgDir.toStdString(),
                 cloudFolder = cloudDir.toStdString(), count, calib]() {
        try {
            tp::BGADetector detector;
            std::vector<double> coplanarities;
            std::vector<tp::BallResult> lastResults;
            cv::Mat lastImage;
            Eigen::Vector3d lastNormal{0, 0, -1};
            double lastD = 0;
            int lastNumRows = 0;

            for (int i = 1; i <= count; ++i) {
                emit bgaLogMessage(QStringLiteral("处理第 %1/%2 次测量...")
                    .arg(i).arg(count));

                std::string imgPath = imgFolder + "/" + std::to_string(i) + ".bmp";
                cv::Mat brightImg = imreadSafe(imgPath, cv::IMREAD_GRAYSCALE);
                if (brightImg.empty()) {
                    emit bgaLogMessage(QStringLiteral("无法加载图像 %1, 跳过")
                        .arg(QString::fromStdString(imgPath)));
                    continue;
                }

                // Load point cloud for Z matrix
                std::string cloudPath;
                namespace fs = std::filesystem;
                if (fs::exists(fs::u8path(cloudFolder + "/" + std::to_string(i) + ".ply")))
                    cloudPath = cloudFolder + "/" + std::to_string(i) + ".ply";
                else
                    cloudPath = cloudFolder + "/" + std::to_string(i) + ".pcd";

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
                try {
                    cloud = tp::PointCloudIO::load(cloudPath);
                } catch (const std::exception& ex) {
                    emit bgaLogMessage(QStringLiteral("无法加载点云 %1: %2")
                        .arg(QString::fromStdString(cloudPath)).arg(ex.what()));
                    continue;
                }
                if (!cloud || cloud->empty()) continue;

                // Build Z matrix from ordered point cloud (NaN→0)
                int rows = brightImg.rows, cols = brightImg.cols;
                cv::Mat Z(rows, cols, CV_64F, cv::Scalar(0));
                if (static_cast<int>(cloud->size()) == rows * cols) {
                    for (int r = 0; r < rows; ++r)
                        for (int c = 0; c < cols; ++c) {
                            float z = (*cloud)[r * cols + c].z;
                            Z.at<double>(r, c) = std::isfinite(z) ? static_cast<double>(z) : 0.0;
                        }
                }

                // Use calibration X/Y matrices
                cv::Mat X = calib.X, Y = calib.Y;
                if (X.empty() || Y.empty()) {
                    // Fallback: build from ordered cloud (NaN→0)
                    X = cv::Mat(rows, cols, CV_64F);
                    Y = cv::Mat(rows, cols, CV_64F);
                    for (int r = 0; r < rows; ++r)
                        for (int c = 0; c < cols; ++c) {
                            auto& pt = (*cloud)[r * cols + c];
                            X.at<double>(r, c) = std::isfinite(pt.x) ? static_cast<double>(pt.x) : 0.0;
                            Y.at<double>(r, c) = std::isfinite(pt.y) ? static_cast<double>(pt.y) : 0.0;
                        }
                }

                tp::BGAConfig config;
                auto results = detector.runAllDealTwoplane(brightImg, X, Y, Z, config);
                emit bgaLogMessage(QStringLiteral("检测到 %1 个焊球 (第 %2 次)")
                    .arg(results.size()).arg(i));

                if (results.size() < 3) {
                    emit bgaLogMessage(QStringLiteral("第 %1 次焊球不足3个, 跳过共面度计算").arg(i));
                    continue;
                }

                // Compute coplanarity from heights
                double maxH = 0, minH = 1e9;
                for (const auto& b : results) {
                    if (!b.success) continue;
                    if (b.height > maxH) maxH = b.height;
                    if (b.height < minH) minH = b.height;
                }
                double cop = maxH - minH;
                coplanarities.push_back(cop);
                emit bgaLogMessage(QStringLiteral("第 %1 次共面度 = %2 mm")
                    .arg(i).arg(cop, 0, 'f', 4));

                lastResults = results;
                lastImage = brightImg.clone();
                lastNormal = detector.getSubstrateNormal();
                lastD = detector.getSubstrateD();

                // Count rows
                int maxRow = 0;
                for (const auto& b : results)
                    if (b.row > maxRow) maxRow = b.row;
                lastNumRows = maxRow + 1;
            }

            // Build result text
            QString resultText;
            for (int i = 0; i < static_cast<int>(coplanarities.size()); ++i) {
                resultText += QStringLiteral("测量 %1: 共面度 = %2 mm\n")
                    .arg(i + 1).arg(coplanarities[i], 0, 'f', 4);
            }
            if (!coplanarities.empty()) {
                double sum = 0;
                for (auto v : coplanarities) sum += v;
                double avg = sum / coplanarities.size();
                resultText += QStringLiteral("\n平均共面度 = %1 mm\n").arg(avg, 0, 'f', 4);
            }

            // Add per-ball data table
            resultText += QStringLiteral("\n--- 焊球详细数据 ---\n");
            resultText += QStringLiteral("编号\t高度(mm)\t行号\tX3D\tY3D\tZ3D\n");
            for (const auto& b : lastResults) {
                if (!b.success) continue;
                resultText += QStringLiteral("%1\t%2\t%3\t%4\t%5\t%6\n")
                    .arg(b.order)
                    .arg(b.height, 0, 'f', 4)
                    .arg(b.row)
                    .arg(b.x3d, 0, 'f', 3)
                    .arg(b.y3d, 0, 'f', 3)
                    .arg(b.z3d, 0, 'f', 3);
            }

            // Generate charts (picsrc style)
            std::vector<int> measIndex(coplanarities.size());
            for (int i = 0; i < static_cast<int>(coplanarities.size()); ++i)
                measIndex[i] = i + 1;

            // 仅在工作线程做数据处理；Qt图表绘制放到UI线程，避免线程相关崩溃
            cv::Mat ballOverlayImg;
            if (!lastImage.empty() && !lastResults.empty()) {
                ballOverlayImg = tp::ChartRenderer::renderBallOverlay(lastImage, lastResults);
            }

            // 在工作线程中提前做深拷贝QImage，避免UI线程拿到悬空Mat数据
            QImage ballQImg = matToQImageCopy(ballOverlayImg);

            QMetaObject::invokeMethod(this, [this, resultText, coplanarities,
                    ballQImg, lastResults, lastImage,
                    lastNormal, lastD, lastNumRows]() {
                bgaCoplanarities_ = coplanarities;
                lastBgaResults_ = lastResults;
                lastBgaImage_ = lastImage;
                lastSubstrateNormal_ = lastNormal;
                lastSubstrateD_ = lastD;
                lastBgaNumRows_ = lastNumRows;

                ui->bgaDataText->setPlainText(resultText);

                if (ballQImg.isNull()) {
                    appendBgaLog(QStringLiteral("BGA图像为空: 焊球定位图未生成"));
                } else if (!zoomBgaBall_) {
                    appendBgaLog(QStringLiteral("BGA图像更新失败: 焊球定位控件为空"));
                } else {
                    zoomBgaBall_->setImage(ballQImg);
                    appendBgaLog(QStringLiteral("BGA图像更新: 焊球定位图 %1x%2")
                        .arg(ballQImg.width()).arg(ballQImg.height()));
                }

                // 在UI线程按当前算法结果绘制柱状图+折线图
                updateBgaChartsFromCurrentResults();

                ui->bgaBtnStart->setEnabled(true);
                appendBgaLog(QStringLiteral("测量完成"));
            }, Qt::QueuedConnection);

        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString(e.what())]() {
                appendBgaLog(QStringLiteral("测量错误 - ") + msg);
                ui->bgaBtnStart->setEnabled(true);
            }, Qt::QueuedConnection);
        } catch (...) {
            QMetaObject::invokeMethod(this, [this]() {
                appendBgaLog(QStringLiteral("测量发生未知错误"));
                ui->bgaBtnStart->setEnabled(true);
            }, Qt::QueuedConnection);
        }
    }).detach();
}
void MainWindow::onBgaBtnPlotClicked() {
    if (lastBgaResults_.empty()) {
        appendBgaLog(QStringLiteral("请先执行测量"));
        return;
    }
    // 按当前算法数据重绘柱状图+折线图
    updateBgaChartsFromCurrentResults();
    appendBgaLog(QStringLiteral("图表已刷新"));
}
void MainWindow::onBgaBtnBallShowClicked() {
    if (lastBgaResults_.empty() || lastBgaImage_.empty()) {
        appendBgaLog(QStringLiteral("请先执行测量"));
        return;
    }
    auto orderOverlay = tp::ChartRenderer::renderOrderOverlay(
        lastBgaImage_, lastBgaResults_, lastBgaNumRows_);
    auto qimg = matToQImageCopy(orderOverlay);
    if (!qimg.isNull() && zoomBgaBall_) {
        zoomBgaBall_->setImage(qimg);
        appendBgaLog(QStringLiteral("BGA图像更新: 焊球编号图 %1x%2")
            .arg(qimg.width()).arg(qimg.height()));
    } else if (qimg.isNull()) {
        appendBgaLog(QStringLiteral("BGA图像为空: 焊球编号图重绘失败"));
    } else if (!zoomBgaBall_) {
        appendBgaLog(QStringLiteral("BGA图像更新失败: 焊球编号控件为空"));
    }
    appendBgaLog(QStringLiteral("焊球编号标注已显示"));
}
void MainWindow::onBgaBtnImportClicked() {
    auto path = QFileDialog::getOpenFileName(this,
        QStringLiteral("导入 BGA 数据"), {}, "CSV (*.csv);;TXT (*.txt)");
    if (path.isEmpty()) return;
    appendBgaLog(QStringLiteral("数据导入: ") + path);
}
void MainWindow::onBgaBtnSaveClicked() {
    if (lastBgaResults_.empty()) {
        appendBgaLog(QStringLiteral("请先执行测量"));
        return;
    }
    auto path = QFileDialog::getSaveFileName(this,
        QStringLiteral("导出BGA测量结果"), {}, "CSV (*.csv)");
    if (path.isEmpty()) return;

    try {
        std::ofstream ofs(std::filesystem::u8path(path.toStdString()));
        ofs << "order,xc,yc,radius,x3d,y3d,z3d,height,row,success\n";
        for (const auto& b : lastBgaResults_) {
            ofs << b.order << "," << b.xc << "," << b.yc << ","
                << b.radius << "," << b.x3d << "," << b.y3d << ","
                << b.z3d << "," << b.height << "," << b.row << ","
                << (b.success ? 1 : 0) << "\n";
        }
        appendBgaLog(QStringLiteral("已导出: ") + path);
    } catch (const std::exception& e) {
        appendBgaLog(QStringLiteral("导出失败: ") + e.what());
    }
}

void MainWindow::onBgaBtnImgFolderBrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择BGA图像文件夹"));
    if (!dir.isEmpty()) ui->bgaEditImgFolder->setText(dir);
}

void MainWindow::onBgaBtnCloudFolderBrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择BGA点云文件夹"));
    if (!dir.isEmpty()) ui->bgaEditCloudFolder->setText(dir);
}

// ============================================================
// qfpWidget
// ============================================================

void MainWindow::onQfpSpinCountChanged(int) {
}
void MainWindow::onQfpBtnStartClicked() {
    QString imgDir = ui->qfpEditImgFolder->text().trimmed();
    QString cloudDir = ui->qfpEditCloudFolder->text().trimmed();
    int count = ui->qfpSpinCount->value();

    if (imgDir.isEmpty() || cloudDir.isEmpty()) {
        appendQfpLog(QStringLiteral("请先设置图像文件夹和点云文件夹"));
        return;
    }

    appendQfpLog(QStringLiteral("开始测量 (%1 次)...").arg(count));
    ui->qfpBtnStart->setEnabled(false);

    std::thread([this, imgFolder = imgDir.toStdString(),
                 cloudFolder = cloudDir.toStdString(), count]() {
        try {
            tp::BGAMeasurePipeline pipeline;
            QString resultText;
            cv::Mat lastSegImg;
            std::vector<double> copSeries;
            std::vector<double> maxWarpSeries;
            std::vector<double> avgWarpSeries;
            std::vector<double> maxHeightSeries;

            for (int i = 1; i <= count; ++i) {
                emit qfpLogMessage(QStringLiteral("处理第 %1/%2 次...")
                    .arg(i).arg(count));

                std::string imgPath = imgFolder + "/" + std::to_string(i) + ".bmp";
                cv::Mat gray = imreadSafe(imgPath, cv::IMREAD_GRAYSCALE);
                if (gray.empty()) {
                    emit qfpLogMessage(QStringLiteral("无法加载 %1, 跳过")
                        .arg(QString::fromStdString(imgPath)));
                    continue;
                }

                cv::Mat chipMask;
                auto chipInfo = pipeline.localizeChip(gray, chipMask);
                auto bodyRect = pipeline.refineBodyBoundary(gray, chipMask);
                auto searchRegions = pipeline.defineSearchRegions(chipInfo, gray.size());
                auto pins = pipeline.segmentPins(gray, searchRegions, bodyRect);

                emit qfpLogMessage(QStringLiteral("第 %1 次分割完成, 检测到 %2 个引脚")
                    .arg(i).arg(pins.size()));

                resultText += QStringLiteral("测量 %1: 引脚数 = %2\n")
                    .arg(i).arg(pins.size());

                // QFP图表指标（当前以分割统计值作为测量显示）
                double maxArea = 0.0;
                double sumArea = 0.0;
                double maxHeight = 0.0;
                for (const auto& pin : pins) {
                    maxArea = std::max(maxArea, pin.area);
                    sumArea += pin.area;
                    maxHeight = std::max(maxHeight, static_cast<double>(pin.bbox.height));
                }
                double avgArea = pins.empty() ? 0.0 : (sumArea / pins.size());
                copSeries.push_back(static_cast<double>(pins.size()));
                maxWarpSeries.push_back(maxArea);
                avgWarpSeries.push_back(avgArea);
                maxHeightSeries.push_back(maxHeight);

                cv::Mat segShow;
                cv::cvtColor(gray, segShow, cv::COLOR_GRAY2BGR);
                for (const auto& pin : pins) {
                    cv::rectangle(segShow, pin.bbox, cv::Scalar(0, 255, 0), 1);
                }
                lastSegImg = segShow.clone();
            }

            std::vector<int> xIndex(copSeries.size());
            for (int i = 0; i < static_cast<int>(copSeries.size()); ++i) xIndex[i] = i + 1;

            QImage segQImg = matToQImageCopy(lastSegImg);

            QMetaObject::invokeMethod(this, [this, resultText, segQImg,
                                             copSeries, maxWarpSeries, avgWarpSeries, maxHeightSeries,
                                             lastSegImg]() {
                ui->qfpDataText->setPlainText(resultText);
                qfpCoplanaritySeries_ = copSeries;
                qfpMaxWarpAngleSeries_ = maxWarpSeries;
                qfpAvgWarpAngleSeries_ = avgWarpSeries;
                qfpMaxWarpHeightSeries_ = maxHeightSeries;
                lastQfpSegImage_ = lastSegImg;

                if (!segQImg.isNull() && zoomQfpSeg_)
                    zoomQfpSeg_->setImage(segQImg);

                // 在UI线程生成QFP图表
                if (!qfpCoplanaritySeries_.empty()) {
                    std::vector<int> xIndex(qfpCoplanaritySeries_.size());
                    for (int i = 0; i < static_cast<int>(qfpCoplanaritySeries_.size()); ++i)
                        xIndex[i] = i + 1;

                    cv::Mat copMat = tp::ChartRenderer::renderStyledBarChart(
                        qfpCoplanaritySeries_, xIndex,
                        "QFP Coplanarity", "Measure Index", "Pins",
                        cv::Scalar(243, 150, 33));
                    cv::Mat maxWarpMat = tp::ChartRenderer::renderStyledBarChart(
                        qfpMaxWarpAngleSeries_, xIndex,
                        "QFP Max Warp", "Measure Index", "Max Area(px)",
                        cv::Scalar(176, 39, 156));
                    cv::Mat avgWarpMat = tp::ChartRenderer::renderStyledBarChart(
                        qfpAvgWarpAngleSeries_, xIndex,
                        "QFP Avg Warp", "Measure Index", "Avg Area(px)",
                        cv::Scalar(55, 186, 0));
                    cv::Mat maxHeightMat = tp::ChartRenderer::renderStyledBarChart(
                        qfpMaxWarpHeightSeries_, xIndex,
                        "QFP Max Height", "Measure Index", "Height(px)",
                        cv::Scalar(99, 30, 233));

                    QImage copQImg = matToQImageCopy(copMat);
                    QImage maxWarpQImg = matToQImageCopy(maxWarpMat);
                    QImage avgWarpQImg = matToQImageCopy(avgWarpMat);
                    QImage maxHeightQImg = matToQImageCopy(maxHeightMat);

                    if (!copQImg.isNull() && zoomQfpCoplanar_)
                        zoomQfpCoplanar_->setImage(copQImg);
                    if (!maxWarpQImg.isNull() && zoomQfpMaxWarpAngle_)
                        zoomQfpMaxWarpAngle_->setImage(maxWarpQImg);
                    if (!avgWarpQImg.isNull() && zoomQfpAvgWarpAngle_)
                        zoomQfpAvgWarpAngle_->setImage(avgWarpQImg);
                    if (!maxHeightQImg.isNull() && zoomQfpWarpHeight_)
                        zoomQfpWarpHeight_->setImage(maxHeightQImg);
                }

                ui->qfpBtnStart->setEnabled(true);
                appendQfpLog(QStringLiteral("测量完成"));
            }, Qt::QueuedConnection);

        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString(e.what())]() {
                appendQfpLog(QStringLiteral("测量错误 - ") + msg);
                ui->qfpBtnStart->setEnabled(true);
            }, Qt::QueuedConnection);
        }
    }).detach();
}
void MainWindow::onQfpBtnPlotClicked() {
    if (qfpCoplanaritySeries_.empty()) {
        appendQfpLog(QStringLiteral("请先执行测量"));
        return;
    }
    std::vector<int> xIndex(qfpCoplanaritySeries_.size());
    for (int i = 0; i < static_cast<int>(qfpCoplanaritySeries_.size()); ++i) xIndex[i] = i + 1;

    auto cop = tp::ChartRenderer::renderStyledBarChart(
        qfpCoplanaritySeries_, xIndex,
        "QFP Coplanarity", "Measure Index", "Pins",
        cv::Scalar(243, 150, 33));
    auto maxA = tp::ChartRenderer::renderStyledBarChart(
        qfpMaxWarpAngleSeries_, xIndex,
        "QFP Max Warp", "Measure Index", "Max Area(px)",
        cv::Scalar(176, 39, 156));
    auto avgA = tp::ChartRenderer::renderStyledBarChart(
        qfpAvgWarpAngleSeries_, xIndex,
        "QFP Avg Warp", "Measure Index", "Avg Area(px)",
        cv::Scalar(55, 186, 0));
    auto maxH = tp::ChartRenderer::renderStyledBarChart(
        qfpMaxWarpHeightSeries_, xIndex,
        "QFP Max Height", "Measure Index", "Height(px)",
        cv::Scalar(99, 30, 233));

    if (zoomQfpCoplanar_) zoomQfpCoplanar_->setImage(matToQImageCopy(cop));
    if (zoomQfpMaxWarpAngle_) zoomQfpMaxWarpAngle_->setImage(matToQImageCopy(maxA));
    if (zoomQfpAvgWarpAngle_) zoomQfpAvgWarpAngle_->setImage(matToQImageCopy(avgA));
    if (zoomQfpWarpHeight_) zoomQfpWarpHeight_->setImage(matToQImageCopy(maxH));
    appendQfpLog(QStringLiteral("QFP图表已刷新"));
}
void MainWindow::onQfpBtnPinShowClicked() {
    if (lastQfpSegImage_.empty()) {
        appendQfpLog(QStringLiteral("请先执行测量"));
        return;
    }
    if (zoomQfpSeg_) zoomQfpSeg_->setImage(matToQImageCopy(lastQfpSegImage_));
    appendQfpLog(QStringLiteral("引脚分割结果已显示"));
}
void MainWindow::onQfpBtnImportClicked() {
    auto path = QFileDialog::getOpenFileName(this,
        QStringLiteral("导入 QFP 数据"), {}, "CSV (*.csv);;TXT (*.txt)");
    if (path.isEmpty()) return;
    appendQfpLog(QStringLiteral("数据导入: ") + path);
}
void MainWindow::onQfpBtnSaveClicked() {
    appendQfpLog(QStringLiteral("导出功能开发中"));
}

void MainWindow::onQfpBtnImgFolderBrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择QFP图像文件夹"));
    if (!dir.isEmpty()) ui->qfpEditImgFolder->setText(dir);
}

void MainWindow::onQfpBtnCloudFolderBrowseClicked() {
    auto dir = QFileDialog::getExistingDirectory(this,
        QStringLiteral("选择QFP点云文件夹"));
    if (!dir.isEmpty()) ui->qfpEditCloudFolder->setText(dir);
}
