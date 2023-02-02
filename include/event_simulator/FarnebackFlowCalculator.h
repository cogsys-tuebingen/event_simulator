#pragma once

#include <event_simulator/DenseOpticalFlowCalculator.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * @brief Class implementing the Farneback optical flow calculator.
 */
class FarnebackFlowCalculator : public DenseOpticalFlowCalculator {
public:
  /**
   * @brief Constructor setting the name of the optical flow method.
   */
  FarnebackFlowCalculator();

  /**
   * @brief Calculated the optical flow with the Farneback optical flow
   *        method form OpenCV.
   *
   * @param prev_frame Frame from previous time step
   * @param frame Frame from current time step
   */
  cv::Mat calculateFlow(const cv::Mat prev_frame, const cv::Mat frame) override;

  /**
   * @brief Returns the name of the optical flow method.
   */
  std::string getName() override { return name_; };

private:
  /// Name of the optical flow method
  std::string name_;
};
