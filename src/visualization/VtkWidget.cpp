#include "VtkWidget.h"
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkCamera.h>
#include <vtkAxesActor.h>
#include <vtkProperty.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

namespace tp {

VtkWidget::VtkWidget(QWidget* parent)
    : QVTKOpenGLNativeWidget(parent)
{
    renderWindow_ = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_->SetBackground(0.17, 0.17, 0.17);
    renderWindow_->AddRenderer(renderer_);
    setRenderWindow(renderWindow_);
}

VtkWidget::~VtkWidget() noexcept = default;

void VtkWidget::updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!cloud || cloud->empty()) return;

    auto points = vtkSmartPointer<vtkPoints>::New();
    auto cells  = vtkSmartPointer<vtkCellArray>::New();
    points->SetNumberOfPoints(static_cast<vtkIdType>(cloud->size()));

    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud->size()); ++i) {
        const auto& pt = (*cloud)[i];
        points->SetPoint(i, pt.x, pt.y, pt.z);
        cells->InsertNextCell(1, &i);
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(cells);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    renderer_->RemoveAllViewProps();
    renderer_->AddActor(actor);
    cloudActor_ = actor;
    renderer_->ResetCamera();
    renderWindow_->Render();
}

void VtkWidget::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    if (!cloud || cloud->empty()) return;

    auto points = vtkSmartPointer<vtkPoints>::New();
    auto cells  = vtkSmartPointer<vtkCellArray>::New();
    auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    points->SetNumberOfPoints(static_cast<vtkIdType>(cloud->size()));

    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud->size()); ++i) {
        const auto& pt = (*cloud)[i];
        points->SetPoint(i, pt.x, pt.y, pt.z);
        cells->InsertNextCell(1, &i);
        unsigned char rgb[3] = { pt.r, pt.g, pt.b };
        colors->InsertNextTypedTuple(rgb);
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(cells);
    polyData->GetPointData()->SetScalars(colors);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    renderer_->RemoveAllViewProps();
    renderer_->AddActor(actor);
    cloudActor_ = actor;
    renderer_->ResetCamera();
    renderWindow_->Render();
}

void VtkWidget::clearCloud() {
    renderer_->RemoveAllViewProps();
    renderWindow_->Render();
}

void VtkWidget::resetCamera() {
    renderer_->ResetCamera();
    renderWindow_->Render();
}

void VtkWidget::setBackgroundColor(double r, double g, double b) {
    renderer_->SetBackground(r, g, b);
    renderWindow_->Render();
}

void VtkWidget::setCameraPreset(ViewPreset preset) {
    auto* cam = renderer_->GetActiveCamera();
    cam->SetFocalPoint(0, 0, 0);
    switch (preset) {
        case ViewFront:     cam->SetPosition(0, -1000, 0);   cam->SetViewUp(0, 0, 1); break;
        case ViewBack:      cam->SetPosition(0,  1000, 0);   cam->SetViewUp(0, 0, 1); break;
        case ViewLeft:      cam->SetPosition(-1000, 0, 0);   cam->SetViewUp(0, 0, 1); break;
        case ViewRight:     cam->SetPosition(1000, 0, 0);    cam->SetViewUp(0, 0, 1); break;
        case ViewTop:       cam->SetPosition(0, 0, 1000);    cam->SetViewUp(0, 1, 0); break;
        case ViewBottom:    cam->SetPosition(0, 0, -1000);   cam->SetViewUp(0, 1, 0); break;
        case ViewIsometric: cam->SetPosition(577, 577, 577); cam->SetViewUp(0, 0, 1); break;
    }
    renderer_->ResetCameraClippingRange();
    renderWindow_->Render();
}

void VtkWidget::setGridVisible(bool visible) {
    if (visible && !gridActor_) {
        auto axes = vtkSmartPointer<vtkAxesActor>::New();
        axes->SetTotalLength(100, 100, 100);
        gridActor_ = axes;
        renderer_->AddViewProp(gridActor_);
    }
    if (gridActor_) gridActor_->SetVisibility(visible);
    renderWindow_->Render();
}

void VtkWidget::setWireframe(bool enabled) {
    if (cloudActor_) {
        cloudActor_->GetProperty()->SetRepresentation(
            enabled ? VTK_WIREFRAME : VTK_POINTS);
        renderWindow_->Render();
    }
}

void VtkWidget::saveScreenshot(const std::string& path) {
    auto filter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    filter->SetInput(renderWindow_);
    filter->Update();
    auto writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(path.c_str());
    writer->SetInputConnection(filter->GetOutputPort());
    writer->Write();
}

} // namespace tp
