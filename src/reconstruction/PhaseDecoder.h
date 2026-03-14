#pragma once

#include "common/interfaces.h"
#include <cmath>

namespace tp {

class PhaseDecoder : public IDecoder {
public:
    PhaseDecoder() = default;
    ~PhaseDecoder() noexcept override = default;

    // IDecoder interface
    cv::Mat calcPhaseMatrix(
        const std::vector<cv::Mat>& imgs, int steps) override;
    cv::Mat multiFreqUnwrap(
        const cv::Mat& phi1, const cv::Mat& phi2, const cv::Mat& phi3,
        int f1, int f2, int f3) override;
    cv::Mat calcFringeModulation(
        const std::vector<cv::Mat>& imgs, int steps) override;
    cv::Mat calcModulationContrast(
        const std::vector<cv::Mat>& imgs, int steps) override;
    cv::Mat calcPhaseErrorEnergy(
        const std::vector<cv::Mat>& imgs,
        const cv::Mat& wrappedPhase, int steps) override;
    cv::Mat unwrapPhase(
        const cv::Mat& keyPhase, int multiple,
        const cv::Mat& wrappedPhase) override;

private:
    // helper: compute difference phase [0, 2pi)
    cv::Mat multiFreqDiff(const cv::Mat& pha1, const cv::Mat& pha2);
};

} // namespace tp
