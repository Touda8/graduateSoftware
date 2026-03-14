#pragma once

#include <QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkActor.h>
#include <vtkProp3D.h>
#include <vtkScalarBarActor.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace tp {

class VtkWidget : public QVTKOpenGLNativeWidget {
    Q_OBJECT

public:
    explicit VtkWidget(QWidget* parent = nullptr);
    ~VtkWidget() noexcept override;

    void updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void clearCloud();
    void resetCamera();
    void setBackgroundColor(double r, double g, double b);

    enum ViewPreset { ViewFront, ViewBack, ViewLeft, ViewRight, ViewTop, ViewBottom, ViewIsometric };
    void setCameraPreset(ViewPreset preset);
    void setGridVisible(bool visible);
    void setWireframe(bool enabled);
    void saveScreenshot(const std::string& path);

private:
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow_;
    vtkSmartPointer<vtkProp3D> gridActor_;
    vtkSmartPointer<vtkActor> cloudActor_;
    vtkSmartPointer<vtkScalarBarActor> scalarBar_;
};

} // namespace tp
