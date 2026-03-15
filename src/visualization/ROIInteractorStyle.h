#pragma once

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <functional>
#include <string>

namespace tp {

class ROIInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
    static ROIInteractorStyle* New();
    vtkTypeMacro(ROIInteractorStyle, vtkInteractorStyleTrackballCamera);

    void OnKeyPress() override;
    void OnMouseWheelForward() override;
    void OnMouseWheelBackward() override;

    // Set by VtkWidget when crop mode is active
    std::function<void(const std::string& key)> onKeyCallback;
    std::function<void(double factor)> onScrollCallback;
    bool cropModeActive = false;
};

} // namespace tp
