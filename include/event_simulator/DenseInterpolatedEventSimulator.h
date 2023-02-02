#pragma once

#include <event_simulator/DenseOpticalFlowCalculator.h>
#include <event_simulator/EventSimulator.h>
#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Class impementing a dense interpolation event simulator.
 */
class DenseInterpolatedEventSimulator : public EventSimulator {
public:
  /**
   * @brief Constructor initializing number of interpolated frames, the optical flow
   *        algorithm, the name, and the thresholds.
   *
   * @param optical_flow_calculator Optical flow calculator
   * @param num_inter_frames Number of interpolated frames
   * @param c_pos Positive threshold
   * @param c_neg Negative threshold
   */
  DenseInterpolatedEventSimulator(
      const std::shared_ptr<DenseOpticalFlowCalculator> optical_flow_calculator,
      const int num_inter_frames, const int c_pos, const int c_neg);

  /**
   * @brief Simulates events by first using dense optical flow to interpolate
   *        frames. Afterwards using theresholding to get the simulated events
   */
  std::vector<Event>& getEvents(const cv::Mat prev_frame, const cv::Mat frame,
                                const unsigned int prev_timestamp,
                                const unsigned int timestamp,
                                int& num_frames) override;

  /**
   * @brief Simulates events by first using dense optical flow to interpolate
   *        frames. Afterwards using theresholding to get the simulated events
   */
  std::vector<cv::Mat>& getEventFrame(const cv::Mat prev_frame, const cv::Mat frame,
                                      int& num_frames) override;

  /**
   * @brief Returns the name of the event simulator.
   */
  std::string getName() override {
    std::string optical_flow_calculator_name =
        optical_flow_calculator_->getName();
    return name_ + "_" + optical_flow_calculator_name + "_f_p_n_pi_pn_" +
           std::to_string(num_inter_frames_) + "_" + std::to_string(c_pos_) +
           "_" + std::to_string(c_neg_);
  }

protected:
  /// Number of interpolated frames
  int num_inter_frames_;

  /// Pointer to the optical flow calculator
  std::shared_ptr<DenseOpticalFlowCalculator> optical_flow_calculator_;

private:
  /// Name of the event simulator
  std::string name_;

  /// Positive threshold
  int c_pos_;

  /// Negative threshold
  int c_neg_;
};
