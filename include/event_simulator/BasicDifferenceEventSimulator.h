#pragma once

#include <event_simulator/EventSimulator.h>
#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Class implementing the simple difference frame-based event simulator.
 */
class BasicDifferenceEventSimulator : public EventSimulator {
public:
  /**
   * @brief Constructor initializing name, the positive and negative threshold
   *
   * @param c_pos Positive threshold
   * @param c_neg Negative threshold
   */
  BasicDifferenceEventSimulator(const int c_pos, const int c_neg);

  /**
   * @brief Get darker and brighter pixels by thresholding
   *
   * @return The events
   */
  std::vector<Event>& getEvents(const cv::Mat prev_frame, const cv::Mat frame,
                                const unsigned int prev_timestamp,
                                const unsigned int timestamp,
                                int& num_frames) override;

  /**
   * @brief Get darker and brighter pixels by thresholding
   *
   * @return The accumulated events
   */
  std::vector<cv::Mat>& getEventFrame(const cv::Mat prev_frame, const cv::Mat frame,
                                      int& num_frames) override;

  /**
   * @brief Returns the namee of the event simulator.
   */
  std::string getName() override {
    return name_ + "_" + std::to_string(c_pos_) + "_" + std::to_string(c_neg_);
  }

protected:
  /// Positive threshold
  int c_pos_;

  /// Negative threshold
  int c_neg_;

private:
  /// Name of the event simulator
  std::string name_;
};
