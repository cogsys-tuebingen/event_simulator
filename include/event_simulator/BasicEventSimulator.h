#pragma once

#include <event_simulator/EventSimulator.h>
#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Simulator returning nothing.
 */
class BasicEventSimulator : public EventSimulator {
public:
  /**
   * @brief Constructor initializing the namee.
   */
  BasicEventSimulator();

  /**
   * @brief Clers all events and returns the empty vector.
   */
  std::vector<Event>& getEvents(const cv::Mat prev_frame, const cv::Mat frame,
                                const unsigned int prev_timestamp,
                                const unsigned int timestamp,
                                int& num_frames) override;

  /**
   * @brief Returns the previous frame.
   */
  std::vector<cv::Mat>& getEventFrame(const cv::Mat prev_frame, const cv::Mat frame,
                                      int& num_frames) override;

  /**
   * @brief Returns thee name of the event simulator
   */
  std::string getName() override { return name_; }

private:
  /// Name of the event simulator
  std::string name_;
};
