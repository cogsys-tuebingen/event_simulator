#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * @brief Base class for sparse optical flow calculators
 */
class SparseOpticalFlowCalculator {
public:
  /**
   * @brief Constructor setting the name of the optical flow
   */
  SparseOpticalFlowCalculator();

  virtual std::vector<cv::Point2f>
  /**
   * @brief Calculates the optical flow
   *
   * @param prev_frame Frame from the previous time step
   * @param frame Frame from the current time step
   * @param old_points Points form the prevous calculation
   */
  calculateFlow(cv::Mat prev_frame, cv::Mat frame,
                std::vector<cv::Point2f> old_points) = 0;

  /**
   * @brief Returns the name of the optical flow method
   */
  virtual std::string getName() = 0;

private:
  /// Name of the opical flow method
  std::string name_;
};
