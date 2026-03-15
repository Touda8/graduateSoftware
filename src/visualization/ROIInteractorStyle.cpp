#include "ROIInteractorStyle.h"
#include <vtkRenderWindowInteractor.h>

namespace tp {

vtkStandardNewMacro(ROIInteractorStyle);

void ROIInteractorStyle::OnKeyPress() {
    if (cropModeActive && onKeyCallback) {
        std::string key = this->GetInteractor()->GetKeySym();
        // Only intercept crop keys, pass others through
        if (key == "w" || key == "a" || key == "s" || key == "d" ||
            key == "q" || key == "e" || key == "r" || key == "c" ||
            key == "p" || key == "1" || key == "2") {
            onKeyCallback(key);
            return;  // consume the event
        }
    }
    // Pass through to default camera controls
    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void ROIInteractorStyle::OnMouseWheelForward() {
    if (cropModeActive && onScrollCallback) {
        onScrollCallback(1.1);  // scale up by 10%
    }
    // Always forward to camera zoom
    vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
}

void ROIInteractorStyle::OnMouseWheelBackward() {
    if (cropModeActive && onScrollCallback) {
        onScrollCallback(0.9);  // scale down by 10%
    }
    // Always forward to camera zoom
    vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
}

} // namespace tp
