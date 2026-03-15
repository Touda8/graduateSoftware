#include "VtkWidget.h"
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>
#include <vtkTextProperty.h>
#include <vtkCamera.h>
#include <vtkAxesActor.h>
#include <vtkProperty.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkTransform.h>
#include <vtkPlanes.h>
#include <limits>
#include <cmath>

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
    auto zArray = vtkSmartPointer<vtkFloatArray>::New();
    zArray->SetNumberOfComponents(1);
    zArray->SetName("Depth");

    points->SetNumberOfPoints(static_cast<vtkIdType>(cloud->size()));

    float zMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::lowest();

    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud->size()); ++i) {
        const auto& pt = (*cloud)[i];
        if (!std::isfinite(pt.z)) continue;
        points->SetPoint(i, pt.x, pt.y, pt.z);
        cells->InsertNextCell(1, &i);
        zArray->InsertNextValue(pt.z);
        if (pt.z < zMin) zMin = pt.z;
        if (pt.z > zMax) zMax = pt.z;
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(cells);
    polyData->GetPointData()->SetScalars(zArray);

    // JET-like lookup table
    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetTableRange(zMin, zMax);
    lut->SetHueRange(0.667, 0.0); // blue→red
    lut->SetSaturationRange(1.0, 1.0);
    lut->SetValueRange(1.0, 1.0);
    lut->Build();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
    mapper->SetLookupTable(lut);
    mapper->SetScalarRange(zMin, zMax);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(2);

    // Scalar bar (colorbar)
    if (scalarBar_) renderer_->RemoveActor2D(scalarBar_);
    scalarBar_ = vtkSmartPointer<vtkScalarBarActor>::New();
    scalarBar_->SetLookupTable(lut);
    scalarBar_->SetTitle("Z (mm)");
    scalarBar_->SetNumberOfLabels(5);
    scalarBar_->GetTitleTextProperty()->SetFontSize(12);
    scalarBar_->GetLabelTextProperty()->SetFontSize(10);
    scalarBar_->SetWidth(0.08);
    scalarBar_->SetHeight(0.6);
    scalarBar_->SetPosition(0.9, 0.2);

    renderer_->RemoveAllViewProps();
    renderer_->AddActor(actor);
    renderer_->AddActor2D(scalarBar_);
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
    if (scalarBar_) { renderer_->RemoveActor2D(scalarBar_); scalarBar_ = nullptr; }
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

void VtkWidget::enableBoxCrop(bool enable) {
    if (enable) {
        if (!boxWidget_) {
            boxWidget_ = vtkSmartPointer<vtkBoxWidget2>::New();
            auto* rep = vtkBoxRepresentation::SafeDownCast(boxWidget_->GetRepresentation());
            if (rep) {
                rep->SetPlaceFactor(1.0);
                rep->GetOutlineProperty()->SetColor(1.0, 1.0, 0.0);
                rep->GetOutlineProperty()->SetLineWidth(2.0);
            }
            boxWidget_->SetInteractor(renderWindow_->GetInteractor());
        }
        if (cloudActor_) {
            double bounds[6];
            cloudActor_->GetBounds(bounds);
            auto* rep = vtkBoxRepresentation::SafeDownCast(boxWidget_->GetRepresentation());
            if (rep) rep->PlaceWidget(bounds);
        }
        boxWidget_->On();
        renderWindow_->Render();
    } else {
        if (boxWidget_) {
            boxWidget_->Off();
            renderWindow_->Render();
        }
    }
}

bool VtkWidget::isBoxCropEnabled() const {
    return boxWidget_ && boxWidget_->GetEnabled();
}

std::array<double, 6> VtkWidget::getBoxBounds() const {
    std::array<double, 6> bounds = {0, 0, 0, 0, 0, 0};
    if (!boxWidget_) return bounds;
    auto* rep = vtkBoxRepresentation::SafeDownCast(boxWidget_->GetRepresentation());
    if (!rep) return bounds;

    auto planes = vtkSmartPointer<vtkPlanes>::New();
    rep->GetPlanes(planes);

    // 从 box transform 获取 bounds
    auto transform = vtkSmartPointer<vtkTransform>::New();
    rep->GetTransform(transform);

    // 获取 box representation 的实际 bounds
    double* b = rep->GetBounds();
    bounds = {b[0], b[1], b[2], b[3], b[4], b[5]};
    return bounds;
}

} // namespace tp
