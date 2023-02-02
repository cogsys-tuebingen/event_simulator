#pragma once

#include <event_simulator/SparseOpticalFlowCalculator.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * @brief Class implementing the Lukas Kanade optical flow calculator.
 */
class LKOpticalFlowCalculator : public SparseOpticalFlowCalculator {
public:
  /**
   * @brief Constructor setting the name, max level and windows size.
   *
   * @param max_level Maximum level
   * @param window_size Window size
   */
  LKOpticalFlowCalculator(const int max_level = 3, const int window_size = 15);

  /**
   * @brief Calculates the flow with the Lukas Canade method from OpenCV.
   *
   * @param prev_frame Frame from the previous time step
   * @param frame Frame from the current time step
   * @param old_points Points from the previous calculation
   */
  std::vector<cv::Point2f>
  calculateFlow(cv::Mat prev_frame, cv::Mat frame,
                std::vector<cv::Point2f> old_points) override;

  /**
   * @brief Returns the name of the optical flow method.
   */
  std::string getName() override { return name_; };

private:
  /// Maximum level
  int max_level_;

  /// Window size
  int window_size_;

  /// Name of the optical flow method
  std::string name_;
};
