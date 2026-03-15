#include "VtkWidget.h"
#include "ROIInteractorStyle.h"
#include "measurement/BGADetector.h"

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
#include <vtkOutlineSource.h>
#include <vtkSphereSource.h>
#include <vtkPlaneSource.h>
#include <vtkTextActor.h>
#include <vtkGlyph3D.h>
#include <limits>
#include <cmath>

namespace tp {

// ======================================================================
// Constructor / Destructor
// ======================================================================

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

// ======================================================================
// Internal helpers
// ======================================================================

vtkSmartPointer<vtkActor> VtkWidget::buildRGBActor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto cells  = vtkSmartPointer<vtkCellArray>::New();
    auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    vtkIdType validCount = 0;
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud->size()); ++i) {
        const auto& pt = (*cloud)[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;
        points->InsertNextPoint(pt.x, pt.y, pt.z);
        vtkIdType id = validCount++;
        cells->InsertNextCell(1, &id);
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
    actor->GetProperty()->SetPointSize(2);
    return actor;
}

vtkSmartPointer<vtkActor> VtkWidget::buildHeightActor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    vtkSmartPointer<vtkScalarBarActor>& barOut)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto cells  = vtkSmartPointer<vtkCellArray>::New();
    auto zArray = vtkSmartPointer<vtkFloatArray>::New();
    zArray->SetNumberOfComponents(1);
    zArray->SetName("Depth");

    float zMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::lowest();

    vtkIdType validCount = 0;
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud->size()); ++i) {
        const auto& pt = (*cloud)[i];
        if (!std::isfinite(pt.z)) continue;
        points->InsertNextPoint(pt.x, pt.y, pt.z);
        vtkIdType id = validCount++;
        cells->InsertNextCell(1, &id);
        zArray->InsertNextValue(pt.z);
        if (pt.z < zMin) zMin = pt.z;
        if (pt.z > zMax) zMax = pt.z;
    }

    if (zMin >= zMax) { zMin -= 0.5f; zMax += 0.5f; }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(cells);
    polyData->GetPointData()->SetScalars(zArray);

    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetTableRange(zMin, zMax);
    lut->SetHueRange(0.667, 0.0);
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

    barOut = vtkSmartPointer<vtkScalarBarActor>::New();
    barOut->SetLookupTable(lut);
    barOut->SetTitle("Z (mm)");
    barOut->SetNumberOfLabels(5);
    barOut->GetTitleTextProperty()->SetFontSize(12);
    barOut->GetLabelTextProperty()->SetFontSize(10);
    barOut->SetWidth(0.08);
    barOut->SetHeight(0.6);
    barOut->SetPosition(0.9, 0.2);

    return actor;
}

void VtkWidget::renderScene() {
    renderWindow_->Render();
}

void VtkWidget::reAddGrid() {
    if (gridVisible_ && gridActor_) {
        if (!renderer_->HasViewProp(gridActor_))
            renderer_->AddViewProp(gridActor_);
        gridActor_->SetVisibility(true);
    }
}

void VtkWidget::updateROIParamsText() {
    if (!roiParamsText_ || !boxWidget_ || !boxWidget_->GetEnabled()) return;
    auto* rep = vtkBoxRepresentation::SafeDownCast(boxWidget_->GetRepresentation());
    if (!rep) return;

    double bounds[6];
    double* b = rep->GetBounds();
    for (int i = 0; i < 6; ++i) bounds[i] = b[i];

    char buf[256];
    snprintf(buf, sizeof(buf),
        "ROI: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]\n"
        "Mode: %s",
        bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5],
        obbMode_ ? "OBB" : "AABB");
    roiParamsText_->SetInput(buf);
}

// ======================================================================
// Multi-cloud API
// ======================================================================

void VtkWidget::addOrUpdateCloud(const std::string& id,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                  bool heightColored)
{
    if (!cloud || cloud->empty()) return;

    // If switching from legacy mode, clean up legacy actors
    if (legacyMode_) {
        if (legacyCloudActor_) renderer_->RemoveActor(legacyCloudActor_);
        if (legacyScalarBar_) renderer_->RemoveActor2D(legacyScalarBar_);
        legacyCloudActor_ = nullptr;
        legacyScalarBar_ = nullptr;
        legacyMode_ = false;
    }

    // Remove existing actors for this id
    auto it = cloudActors_.find(id);
    if (it != cloudActors_.end()) {
        renderer_->RemoveActor(it->second.pointActor);
        if (it->second.scalarBar) renderer_->RemoveActor2D(it->second.scalarBar);
        cloudActors_.erase(it);
    }

    CloudActors ca;
    ca.heightColored = heightColored;

    if (heightColored) {
        ca.pointActor = buildHeightActor(cloud, ca.scalarBar);
    } else {
        ca.pointActor = buildRGBActor(cloud);
    }

    renderer_->AddActor(ca.pointActor);
    if (ca.scalarBar) renderer_->AddActor2D(ca.scalarBar);
    cloudActors_[id] = ca;

    reAddGrid();
    renderer_->ResetCamera();
    renderScene();
}

void VtkWidget::removeCloud(const std::string& id) {
    auto it = cloudActors_.find(id);
    if (it == cloudActors_.end()) return;
    renderer_->RemoveActor(it->second.pointActor);
    if (it->second.scalarBar) renderer_->RemoveActor2D(it->second.scalarBar);
    cloudActors_.erase(it);

    // Also remove bbox if it was for this cloud
    if (bboxCloudId_ == id) hideBoundingBox();

    renderScene();
}

void VtkWidget::removeAllClouds() {
    for (auto& [id, ca] : cloudActors_) {
        renderer_->RemoveActor(ca.pointActor);
        if (ca.scalarBar) renderer_->RemoveActor2D(ca.scalarBar);
    }
    cloudActors_.clear();
    hideBoundingBox();
    renderScene();
}

void VtkWidget::setCloudVisible(const std::string& id, bool visible) {
    auto it = cloudActors_.find(id);
    if (it == cloudActors_.end()) return;
    it->second.pointActor->SetVisibility(visible);
    if (it->second.scalarBar) it->second.scalarBar->SetVisibility(visible);
    renderScene();
}

void VtkWidget::setCloudHeightColored(const std::string& id,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                       bool heightColored)
{
    auto it = cloudActors_.find(id);
    if (it == cloudActors_.end()) return;

    bool wasVisible = it->second.pointActor->GetVisibility();

    // Remove old actors
    renderer_->RemoveActor(it->second.pointActor);
    if (it->second.scalarBar) renderer_->RemoveActor2D(it->second.scalarBar);

    // Build new
    CloudActors ca;
    ca.heightColored = heightColored;
    if (heightColored) {
        ca.pointActor = buildHeightActor(cloud, ca.scalarBar);
    } else {
        ca.pointActor = buildRGBActor(cloud);
    }
    ca.pointActor->SetVisibility(wasVisible);
    if (ca.scalarBar) ca.scalarBar->SetVisibility(wasVisible);

    renderer_->AddActor(ca.pointActor);
    if (ca.scalarBar) renderer_->AddActor2D(ca.scalarBar);
    cloudActors_[id] = ca;

    reAddGrid();
    renderScene();
}

// ======================================================================
// Selection bounding box
// ======================================================================

void VtkWidget::showBoundingBox(const std::string& id) {
    hideBoundingBox();

    auto it = cloudActors_.find(id);
    if (it == cloudActors_.end()) return;

    double bounds[6];
    it->second.pointActor->GetBounds(bounds);

    auto outline = vtkSmartPointer<vtkOutlineSource>::New();
    outline->SetBounds(bounds);
    outline->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(outline->GetOutputPort());

    bboxActor_ = vtkSmartPointer<vtkActor>::New();
    bboxActor_->SetMapper(mapper);
    bboxActor_->GetProperty()->SetColor(1.0, 1.0, 0.0); // yellow
    bboxActor_->GetProperty()->SetLineWidth(2.0);

    renderer_->AddActor(bboxActor_);
    bboxCloudId_ = id;
    renderScene();
}

void VtkWidget::hideBoundingBox() {
    if (bboxActor_) {
        renderer_->RemoveActor(bboxActor_);
        bboxActor_ = nullptr;
    }
    bboxCloudId_.clear();
    renderScene();
}

// ======================================================================
// Legacy single-cloud API
// ======================================================================

void VtkWidget::updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!cloud || cloud->empty()) return;
    legacyMode_ = true;

    // Remove multi-cloud actors
    for (auto& [id, ca] : cloudActors_) {
        renderer_->RemoveActor(ca.pointActor);
        if (ca.scalarBar) renderer_->RemoveActor2D(ca.scalarBar);
    }
    cloudActors_.clear();
    hideBoundingBox();

    // Remove previous legacy actors
    if (legacyCloudActor_) renderer_->RemoveActor(legacyCloudActor_);
    if (legacyScalarBar_) renderer_->RemoveActor2D(legacyScalarBar_);

    auto points = vtkSmartPointer<vtkPoints>::New();
    auto cells  = vtkSmartPointer<vtkCellArray>::New();
    auto zArray = vtkSmartPointer<vtkFloatArray>::New();
    zArray->SetNumberOfComponents(1);
    zArray->SetName("Depth");

    float zMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::lowest();

    vtkIdType validCount = 0;
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud->size()); ++i) {
        const auto& pt = (*cloud)[i];
        if (!std::isfinite(pt.z)) continue;
        points->InsertNextPoint(pt.x, pt.y, pt.z);
        vtkIdType id = validCount++;
        cells->InsertNextCell(1, &id);
        zArray->InsertNextValue(pt.z);
        if (pt.z < zMin) zMin = pt.z;
        if (pt.z > zMax) zMax = pt.z;
    }

    if (zMin >= zMax) { zMin -= 0.5f; zMax += 0.5f; }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(cells);
    polyData->GetPointData()->SetScalars(zArray);

    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetTableRange(zMin, zMax);
    lut->SetHueRange(0.667, 0.0);
    lut->SetSaturationRange(1.0, 1.0);
    lut->SetValueRange(1.0, 1.0);
    lut->Build();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
    mapper->SetLookupTable(lut);
    mapper->SetScalarRange(zMin, zMax);

    legacyCloudActor_ = vtkSmartPointer<vtkActor>::New();
    legacyCloudActor_->SetMapper(mapper);
    legacyCloudActor_->GetProperty()->SetPointSize(2);

    legacyScalarBar_ = vtkSmartPointer<vtkScalarBarActor>::New();
    legacyScalarBar_->SetLookupTable(lut);
    legacyScalarBar_->SetTitle("Z (mm)");
    legacyScalarBar_->SetNumberOfLabels(5);
    legacyScalarBar_->GetTitleTextProperty()->SetFontSize(12);
    legacyScalarBar_->GetLabelTextProperty()->SetFontSize(10);
    legacyScalarBar_->SetWidth(0.08);
    legacyScalarBar_->SetHeight(0.6);
    legacyScalarBar_->SetPosition(0.9, 0.2);

    renderer_->AddActor(legacyCloudActor_);
    renderer_->AddActor2D(legacyScalarBar_);
    reAddGrid();
    renderer_->ResetCamera();
    renderScene();
}

void VtkWidget::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    if (!cloud || cloud->empty()) return;
    legacyMode_ = true;

    // Remove multi-cloud actors
    for (auto& [id, ca] : cloudActors_) {
        renderer_->RemoveActor(ca.pointActor);
        if (ca.scalarBar) renderer_->RemoveActor2D(ca.scalarBar);
    }
    cloudActors_.clear();
    hideBoundingBox();

    // Remove previous legacy actors
    if (legacyCloudActor_) renderer_->RemoveActor(legacyCloudActor_);
    if (legacyScalarBar_) { renderer_->RemoveActor2D(legacyScalarBar_); legacyScalarBar_ = nullptr; }

    legacyCloudActor_ = buildRGBActor(cloud);
    renderer_->AddActor(legacyCloudActor_);
    reAddGrid();
    renderer_->ResetCamera();
    renderScene();
}

void VtkWidget::clearCloud() {
    // Clear legacy
    if (legacyCloudActor_) { renderer_->RemoveActor(legacyCloudActor_); legacyCloudActor_ = nullptr; }
    if (legacyScalarBar_) { renderer_->RemoveActor2D(legacyScalarBar_); legacyScalarBar_ = nullptr; }
    legacyMode_ = false;

    // Clear multi-cloud
    removeAllClouds();
    renderScene();
}

// ======================================================================
// Camera & display
// ======================================================================

void VtkWidget::resetCamera() {
    renderer_->ResetCamera();
    renderScene();
}

void VtkWidget::setBackgroundColor(double r, double g, double b) {
    renderer_->SetBackground(r, g, b);
    renderScene();
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
    renderScene();
}

void VtkWidget::setGridVisible(bool visible) {
    gridVisible_ = visible;
    if (visible && !gridActor_) {
        auto axes = vtkSmartPointer<vtkAxesActor>::New();
        axes->SetTotalLength(100, 100, 100);
        gridActor_ = axes;
        renderer_->AddViewProp(gridActor_);
    }
    if (gridActor_) gridActor_->SetVisibility(visible);
    renderScene();
}

void VtkWidget::setWireframe(bool enabled) {
    // Apply to all multi-cloud actors
    for (auto& [id, ca] : cloudActors_) {
        if (ca.pointActor) {
            ca.pointActor->GetProperty()->SetRepresentation(
                enabled ? VTK_WIREFRAME : VTK_POINTS);
        }
    }
    // Also legacy
    if (legacyCloudActor_) {
        legacyCloudActor_->GetProperty()->SetRepresentation(
            enabled ? VTK_WIREFRAME : VTK_POINTS);
    }
    renderScene();
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

// ======================================================================
// ROI crop
// ======================================================================

void VtkWidget::enableROICrop(bool enable, bool obbMode) {
    obbMode_ = obbMode;

    if (enable) {
        if (!boxWidget_) {
            boxWidget_ = vtkSmartPointer<vtkBoxWidget2>::New();
            auto* rep = vtkBoxRepresentation::SafeDownCast(
                boxWidget_->GetRepresentation());
            if (rep) {
                rep->SetPlaceFactor(1.0);
                rep->GetOutlineProperty()->SetColor(1.0, 0.0, 0.0); // red
                rep->GetOutlineProperty()->SetLineWidth(2.0);
            }
            boxWidget_->SetInteractor(renderWindow_->GetInteractor());
        }

        // Place box at the bounds of the first visible cloud or legacy actor
        double bounds[6] = {-1, 1, -1, 1, -1, 1};
        bool found = false;
        for (auto& [id, ca] : cloudActors_) {
            if (ca.pointActor && ca.pointActor->GetVisibility()) {
                ca.pointActor->GetBounds(bounds);
                found = true;
                break;
            }
        }
        if (!found && legacyCloudActor_) {
            legacyCloudActor_->GetBounds(bounds);
        }

        auto* rep = vtkBoxRepresentation::SafeDownCast(
            boxWidget_->GetRepresentation());
        if (rep) rep->PlaceWidget(bounds);
        boxWidget_->On();

        // Install ROI interactor style
        auto style = vtkSmartPointer<ROIInteractorStyle>::New();
        style->cropModeActive = true;
        style->onKeyCallback = [this](const std::string& key) {
            handleROIKey(key);
        };
        style->onScrollCallback = [this](double factor) {
            handleROIScroll(factor);
        };
        renderWindow_->GetInteractor()->SetInteractorStyle(style);

        // Create params text overlay
        if (!roiParamsText_) {
            roiParamsText_ = vtkSmartPointer<vtkTextActor>::New();
            roiParamsText_->GetTextProperty()->SetFontSize(14);
            roiParamsText_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
            roiParamsText_->SetPosition(10, 10);
            renderer_->AddActor2D(roiParamsText_);
        }
        updateROIParamsText();
        renderScene();
    } else {
        if (boxWidget_) {
            boxWidget_->Off();
        }
        if (roiParamsText_) {
            renderer_->RemoveActor2D(roiParamsText_);
            roiParamsText_ = nullptr;
        }
        // Restore default interactor style
        auto style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        renderWindow_->GetInteractor()->SetInteractorStyle(style);
        renderScene();
    }
}

bool VtkWidget::isROICropEnabled() const {
    return boxWidget_ && boxWidget_->GetEnabled();
}

void VtkWidget::resetROIBox() {
    if (!boxWidget_) return;
    double bounds[6] = {-1, 1, -1, 1, -1, 1};
    for (auto& [id, ca] : cloudActors_) {
        if (ca.pointActor && ca.pointActor->GetVisibility()) {
            ca.pointActor->GetBounds(bounds);
            break;
        }
    }
    auto* rep = vtkBoxRepresentation::SafeDownCast(
        boxWidget_->GetRepresentation());
    if (rep) rep->PlaceWidget(bounds);
    updateROIParamsText();
    renderScene();
}

void VtkWidget::getROITransform(double bounds[6]) const {
    if (!boxWidget_) {
        for (int i = 0; i < 6; ++i) bounds[i] = 0;
        return;
    }
    auto* rep = vtkBoxRepresentation::SafeDownCast(
        boxWidget_->GetRepresentation());
    if (!rep) return;
    double* b = rep->GetBounds();
    for (int i = 0; i < 6; ++i) bounds[i] = b[i];
}

void VtkWidget::handleROIKey(const std::string& key) {
    if (!boxWidget_ || !boxWidget_->GetEnabled()) return;

    auto* rep = vtkBoxRepresentation::SafeDownCast(
        boxWidget_->GetRepresentation());
    if (!rep) return;

    if (key == "c") {
        emit roiCropRequested();
        return;
    }
    if (key == "p") {
        emit roiParamsSaveRequested();
        return;
    }
    if (key == "r") {
        resetROIBox();
        return;
    }
    if (key == "1") {
        obbMode_ = false;
        updateROIParamsText();
        renderScene();
        return;
    }
    if (key == "2") {
        obbMode_ = true;
        updateROIParamsText();
        renderScene();
        return;
    }

    // Translation via WASD/QE
    auto transform = vtkSmartPointer<vtkTransform>::New();
    rep->GetTransform(transform);

    constexpr double STEP = 0.5;
    double dx = 0, dy = 0, dz = 0;
    if (key == "d")      dx =  STEP;
    else if (key == "a") dx = -STEP;
    else if (key == "w") dy =  STEP;
    else if (key == "s") dy = -STEP;
    else if (key == "e") dz =  STEP;
    else if (key == "q") dz = -STEP;

    transform->Translate(dx, dy, dz);
    rep->SetTransform(transform);
    updateROIParamsText();
    renderScene();
}

void VtkWidget::handleROIScroll(double factor) {
    if (!boxWidget_ || !boxWidget_->GetEnabled()) return;

    auto* rep = vtkBoxRepresentation::SafeDownCast(
        boxWidget_->GetRepresentation());
    if (!rep) return;

    auto transform = vtkSmartPointer<vtkTransform>::New();
    rep->GetTransform(transform);
    transform->Scale(factor, factor, factor);
    rep->SetTransform(transform);
    updateROIParamsText();
    renderScene();
}

// ======================================================================
// BGA 3D scatter
// ======================================================================

void VtkWidget::showBgaScatter(
    const std::vector<BallResult>& balls,
    const Eigen::Vector3d& substrateNormal,
    double substrateD)
{
    // Switch to legacy mode for BGA visualization
    legacyMode_ = true;

    // Clear everything
    for (auto& [id, ca] : cloudActors_) {
        renderer_->RemoveActor(ca.pointActor);
        if (ca.scalarBar) renderer_->RemoveActor2D(ca.scalarBar);
    }
    cloudActors_.clear();
    if (legacyCloudActor_) { renderer_->RemoveActor(legacyCloudActor_); legacyCloudActor_ = nullptr; }
    if (legacyScalarBar_) { renderer_->RemoveActor2D(legacyScalarBar_); legacyScalarBar_ = nullptr; }

    if (balls.empty()) { renderScene(); return; }

    // Collect valid ball positions and heights
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto heightArr = vtkSmartPointer<vtkFloatArray>::New();
    heightArr->SetNumberOfComponents(1);
    heightArr->SetName("Height");

    double hMin = std::numeric_limits<double>::max();
    double hMax = std::numeric_limits<double>::lowest();

    for (const auto& b : balls) {
        if (!b.success) continue;
        points->InsertNextPoint(b.x3d, b.y3d, b.z3d);
        heightArr->InsertNextValue(static_cast<float>(b.height));
        if (b.height < hMin) hMin = b.height;
        if (b.height > hMax) hMax = b.height;
    }
    if (points->GetNumberOfPoints() == 0) { renderScene(); return; }

    if (hMin >= hMax) { hMin -= 0.01; hMax += 0.01; }

    // Spheres at each ball location
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(heightArr);

    auto sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetRadius(0.05);
    sphere->SetThetaResolution(12);
    sphere->SetPhiResolution(12);

    auto glyph = vtkSmartPointer<vtkGlyph3D>::New();
    glyph->SetInputData(polyData);
    glyph->SetSourceConnection(sphere->GetOutputPort());
    glyph->SetScaleModeToDataScalingOff();

    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetTableRange(hMin, hMax);
    lut->SetHueRange(0.667, 0.0);
    lut->Build();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyph->GetOutputPort());
    mapper->SetLookupTable(lut);
    mapper->SetScalarRange(hMin, hMax);

    legacyCloudActor_ = vtkSmartPointer<vtkActor>::New();
    legacyCloudActor_->SetMapper(mapper);
    renderer_->AddActor(legacyCloudActor_);

    // Scalar bar
    legacyScalarBar_ = vtkSmartPointer<vtkScalarBarActor>::New();
    legacyScalarBar_->SetLookupTable(lut);
    legacyScalarBar_->SetTitle("Height (mm)");
    legacyScalarBar_->SetNumberOfLabels(5);
    legacyScalarBar_->GetTitleTextProperty()->SetFontSize(12);
    legacyScalarBar_->GetLabelTextProperty()->SetFontSize(10);
    legacyScalarBar_->SetWidth(0.08);
    legacyScalarBar_->SetHeight(0.6);
    legacyScalarBar_->SetPosition(0.9, 0.2);
    renderer_->AddActor2D(legacyScalarBar_);

    // Substrate plane (semi-transparent)
    if (substrateNormal.squaredNorm() > 0.5) {
        // Compute plane centroid from balls
        Eigen::Vector3d centroid(0, 0, 0);
        int cnt = 0;
        for (const auto& b : balls) {
            if (!b.success) continue;
            centroid += Eigen::Vector3d(b.x3d, b.y3d, b.z3d);
            cnt++;
        }
        if (cnt > 0) centroid /= cnt;

        auto plane = vtkSmartPointer<vtkPlaneSource>::New();
        plane->SetCenter(centroid.x(), centroid.y(), centroid.z());
        plane->SetNormal(substrateNormal.x(), substrateNormal.y(),
                         substrateNormal.z());
        // Make the plane cover the extent of balls
        double extent = 0;
        for (const auto& b : balls) {
            if (!b.success) continue;
            double d = (Eigen::Vector3d(b.x3d, b.y3d, b.z3d) - centroid).norm();
            if (d > extent) extent = d;
        }
        extent *= 1.5;
        plane->SetPoint1(centroid.x() + extent, centroid.y(), centroid.z());
        plane->SetPoint2(centroid.x(), centroid.y() + extent, centroid.z());
        plane->Update();

        auto planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        planeMapper->SetInputConnection(plane->GetOutputPort());

        auto planeActor = vtkSmartPointer<vtkActor>::New();
        planeActor->SetMapper(planeMapper);
        planeActor->GetProperty()->SetColor(0.3, 0.6, 1.0);
        planeActor->GetProperty()->SetOpacity(0.3);
        renderer_->AddActor(planeActor);
    }

    reAddGrid();
    renderer_->ResetCamera();
    renderScene();
}

// ======================================================================
// Legacy box crop (compatibility)
// ======================================================================

void VtkWidget::enableBoxCrop(bool enable) {
    if (enable) {
        if (!boxWidget_) {
            boxWidget_ = vtkSmartPointer<vtkBoxWidget2>::New();
            auto* rep = vtkBoxRepresentation::SafeDownCast(
                boxWidget_->GetRepresentation());
            if (rep) {
                rep->SetPlaceFactor(1.0);
                rep->GetOutlineProperty()->SetColor(1.0, 1.0, 0.0);
                rep->GetOutlineProperty()->SetLineWidth(2.0);
            }
            boxWidget_->SetInteractor(renderWindow_->GetInteractor());
        }
        // Use legacy or first multi-cloud actor bounds
        vtkActor* targetActor = legacyCloudActor_;
        if (!targetActor) {
            for (auto& [id, ca] : cloudActors_) {
                if (ca.pointActor) { targetActor = ca.pointActor; break; }
            }
        }
        if (targetActor) {
            double bounds[6];
            targetActor->GetBounds(bounds);
            auto* rep = vtkBoxRepresentation::SafeDownCast(
                boxWidget_->GetRepresentation());
            if (rep) rep->PlaceWidget(bounds);
        }
        boxWidget_->On();
        renderScene();
    } else {
        if (boxWidget_) {
            boxWidget_->Off();
            renderScene();
        }
    }
}

bool VtkWidget::isBoxCropEnabled() const {
    return boxWidget_ && boxWidget_->GetEnabled();
}

std::array<double, 6> VtkWidget::getBoxBounds() const {
    std::array<double, 6> bounds = {0, 0, 0, 0, 0, 0};
    if (!boxWidget_) return bounds;
    auto* rep = vtkBoxRepresentation::SafeDownCast(
        boxWidget_->GetRepresentation());
    if (!rep) return bounds;

    double* b = rep->GetBounds();
    bounds = {b[0], b[1], b[2], b[3], b[4], b[5]};
    return bounds;
}

} // namespace tp
