#pragma once

#include <event_simulator/EventSimulator.h>
#include <event_simulator/SparseOpticalFlowCalculator.h>

#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Class implementing the difference frame interpolation
 *        event simulator.
 */
class DifferenceInterpolatedEventSimulator : public EventSimulator {
 public:
  /**
   * @brief Constructor initializing the number of interpolated frames, the optical
   *        flow calculator, the name andn the thresholds.
   *
   * @param optical_flow_calculator Optical flow calculator
   * @param num_inter_frames Number of interpolated frames
   * @param c_pos Positive threshold (used to threshold the events)
   * @param c_neg Negative threshold (used to threshold the events)
   * @param c_pos_inter Positive intermediate threshold (used to thereshold the intitial difference frames)
   * @param c_neg_inter Negative intermediate threshold (used to thereshold the intitial difference frames)
   */
  DifferenceInterpolatedEventSimulator(
      const std::shared_ptr<SparseOpticalFlowCalculator>
          optiacl_flow_calculator,
      const int num_inter_frames, const int c_pos, const int c_neg,
      const int c_pos_inter, const int c_neg_inter);

  /**
   * @brief Setup
   *
   * @param frame_size Size of the frames.
   */
  void setup(const cv::Size frame_size) override;

  /**
   * @brief Simulates events by interpolating between the first and second
   *        difference frame. From these interpolated difference frames, the
   *        events are determined by thresholding.
   */
  std::vector<cv::Mat>& getEventFrame(const cv::Mat prev_frame,
                                      const cv::Mat frame,
                                      int& num_frames) override;

  /**
   * @brief Simulates events by interpolating between the first and second
   *        difference frame. From these interpolated difference frames, the
   *        events are determined by thresholding.
   */
  std::vector<Event>& getEvents(const cv::Mat prev_frame, const cv::Mat frame,
                                const unsigned int prev_timestamp,
                                const unsigned int timestamp,
                                int& num_frames) override;

  /**
   * @brief Returns the name of the event simulator.
   */
  std::string getName() override {
    std::string optical_flow_calculator_name =
        optical_flow_calculator_->getName();
    return name_ + "_" + optical_flow_calculator_name + "_f_p_n_pi_pn_" +
           std::to_string(num_inter_frames_) + "_" + std::to_string(c_pos_) +
           "_" + std::to_string(c_neg_) + "_" + std::to_string(c_pos_inter_) +
           "_" + std::to_string(c_neg_inter_);
  }

 protected:
  /// Number of interpolated frames
  int num_inter_frames_;

  /// Pointer to the optical flow calculator
  std::shared_ptr<SparseOpticalFlowCalculator> optical_flow_calculator_;

  /// Darker masked cached from previous run
  cv::Mat darker_cached_mask_;

  /// Lighter masked cached from previous run
  cv::Mat lighter_cached_mask_;

 private:
  /// Name of the event simulator
  std::string name_;

  /// Positive threshold
  int c_pos_;

  /// Negative threshold
  int c_neg_;

  /// Positive threshold
  int c_pos_inter_;

  /// Negative threshold
  int c_neg_inter_;
};
