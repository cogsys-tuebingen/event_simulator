#pragma once

#include <event_simulator/DenseOpticalFlowCalculator.h>
#include <opencv2/opencv.hpp>
#include <string>

/// Enum to define four level of qualities
enum DISOpticalFlowQuality {
  LOW = 0,
  MEDIUM = 1,
  HIGH = 2,
  EXTREME = 3,
};

/**
 * @brief DIS Optical flow calculator
 */
class DISOpticalFlowCalculator : public DenseOpticalFlowCalculator {
public:
  /**
   * @brief Constructor setting the name and the quality level.
   *
   * @param quality Quality level of the DIS optical flow
   */
  DISOpticalFlowCalculator(const DISOpticalFlowQuality quality);

  /**
   * @brief Calculate the optical flow with:
   *        Fast Optical Flow using Dense Inverse Search:
   *        https://arxiv.org/pdf/1603.03590.pdf
   *
   *        Code taken and adjusted from: https://github.com/tikroeger/OF_DIS
   *
   * @param prev_frame Frame from previous time step
   * @param frame Frame from current time step
   */
  cv::Mat calculateFlow(cv::Mat prev_frame, cv::Mat frame) override;

  /**
   * @brief returns the name of the optical flow method
   */
  std::string getName() override {
    return name_ + "_" + std::to_string(quality_);
  }

private:
  /// DIS optical flow quality
  DISOpticalFlowQuality quality_;

  /// Name of the optical flow method
  std::string name_;
};
