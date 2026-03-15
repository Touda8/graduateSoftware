#pragma once

#include <QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkActor.h>
#include <vtkProp3D.h>
#include <vtkScalarBarActor.h>
#include <vtkBoxWidget2.h>
#include <vtkBoxRepresentation.h>
#include <vtkTextActor.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace tp {

struct BallResult;

class VtkWidget : public QVTKOpenGLNativeWidget {
    Q_OBJECT

public:
    explicit VtkWidget(QWidget* parent = nullptr);
    ~VtkWidget() noexcept override;

    // ---- Multi-cloud API ----
    void addOrUpdateCloud(const std::string& id,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          bool heightColored = false);
    void removeCloud(const std::string& id);
    void removeAllClouds();
    void setCloudVisible(const std::string& id, bool visible);
    void setCloudHeightColored(const std::string& id,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                bool heightColored);

    // ---- Selection bounding box ----
    void showBoundingBox(const std::string& id);
    void hideBoundingBox();

    // ---- Legacy single-cloud API (for BGA scatter etc.) ----
    void updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void clearCloud();

    // ---- Camera & display ----
    void resetCamera();
    void setBackgroundColor(double r, double g, double b);
    enum ViewPreset { ViewFront, ViewBack, ViewLeft, ViewRight,
                      ViewTop, ViewBottom, ViewIsometric };
    void setCameraPreset(ViewPreset preset);
    void setGridVisible(bool visible);
    void setWireframe(bool enabled);
    void saveScreenshot(const std::string& path);

    // ---- ROI crop ----
    void enableROICrop(bool enable, bool obbMode = false);
    bool isROICropEnabled() const;
    void resetROIBox();
    void getROITransform(double bounds[6]) const;
    void handleROIKey(const std::string& key);
    void handleROIScroll(double factor);

    // ---- BGA 3D scatter ----
    void showBgaScatter(const std::vector<BallResult>& balls,
                        const Eigen::Vector3d& substrateNormal,
                        double substrateD);

    // ---- Legacy box crop (kept for compatibility) ----
    void enableBoxCrop(bool enable);
    bool isBoxCropEnabled() const;
    std::array<double, 6> getBoxBounds() const;

signals:
    void boxCropApplied(double xmin, double xmax, double ymin, double ymax,
                        double zmin, double zmax);
    void roiCropRequested();
    void roiSaveRequested();
    void roiParamsSaveRequested();

private:
    // Per-cloud actor storage
    struct CloudActors {
        vtkSmartPointer<vtkActor> pointActor;
        vtkSmartPointer<vtkScalarBarActor> scalarBar;
        bool heightColored = false;
    };

    // Internal helpers
    vtkSmartPointer<vtkActor> buildRGBActor(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    vtkSmartPointer<vtkActor> buildHeightActor(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        vtkSmartPointer<vtkScalarBarActor>& barOut);
    void renderScene();
    void reAddGrid();
    void updateROIParamsText();

    // Renderer core
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow_;

    // Multi-cloud management
    std::map<std::string, CloudActors> cloudActors_;

    // Legacy single-cloud actor (used by updateCloud legacy API)
    vtkSmartPointer<vtkActor> legacyCloudActor_;
    vtkSmartPointer<vtkScalarBarActor> legacyScalarBar_;
    bool legacyMode_ = false;  // true when using legacy updateCloud

    // Grid/axes
    vtkSmartPointer<vtkProp3D> gridActor_;
    bool gridVisible_ = false;

    // Selection bounding box
    vtkSmartPointer<vtkActor> bboxActor_;
    std::string bboxCloudId_;

    // ROI crop
    vtkSmartPointer<vtkBoxWidget2> boxWidget_;
    vtkSmartPointer<vtkTextActor> roiParamsText_;
    bool obbMode_ = false;
};

} // namespace tp
