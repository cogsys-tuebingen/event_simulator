#pragma once

#include <event_simulator/SparseOpticalFlowCalculator.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * @brief Class implementing the Lukas Kanade optical flow calculator
 *        with CUDA support.
 */
class CudaLKOpticalFlowCalculator : public SparseOpticalFlowCalculator {
 public:
  /**
   * @brief Constructor initializing the max level and the window size.
   *
   * @param max_lvl Maximal level
   * @param window_size Window size
   */
  CudaLKOpticalFlowCalculator(const int max_lvl = 3,
                              const int window_size = 15);

  /**
   * @brief Calculates the flow with the Lukas Canade method from OpenCV.
   *
   * @param prev_frame Frame from the previous time step
   * @param frame Frame from the current time step
   * @param old_points Points from the previous calculation
   */
  std::vector<cv::Point2f> calculateFlow(
      const cv::Mat prev_frame, const cv::Mat frame,
      const std::vector<cv::Point2f> old_points) override;

  /**
   * @brief Returns the name of the optical flow method.
   */
  std::string getName() override { return name_; }

 protected:
  /// Maximum level
  int max_lvl_;

  /// Window size
  int window_size_;

 private:
  /// Name of the optical flow method
  std::string name_;
};
