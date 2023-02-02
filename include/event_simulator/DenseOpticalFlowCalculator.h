#pragma once

#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Base class for dense optical flow calculators
 */
class DenseOpticalFlowCalculator {
public:
  /**
   * @brief Constructor setting it's name
   */
  DenseOpticalFlowCalculator();

  /**
   * @brief Calculate optical flow
   *
   * @param prev_frame The frame from the previous time step
   * @param frame The fram from the current time step
   */
  virtual cv::Mat calculateFlow(cv::Mat prev_frame, cv::Mat frame) = 0;

  /**
   * @brief Return the name of the opitcal flow calculator
   */
  virtual std::string getName() = 0;

private:
  /// Name of the optical flow calculator
  std::string name_;
};
