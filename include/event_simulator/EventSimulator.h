#pragma once

#include <opencv2/opencv.hpp>
#include <string>

/**
 * Event struct
 */
struct Event {
  Event(int in_x, int in_y, int in_timestamp, bool in_polarity)
      : x{in_x}, y{in_y}, timestamp{in_timestamp}, polarity{in_polarity} {}
  int x;
  int y;
  int timestamp;
  bool polarity;
};

/**
 * @brief Helper function to pack data into an event struct.
 *
 * @param lighter_events cv::Mat containing pixel which got brighter
 * @param darker_events cv::Mat containing pixel which got darker
 * @param output Vector with events
 * @param timestamp Time stamp of the events
 */
void packEvents(const cv::Mat &lighter_events, const cv::Mat &darker_events,
                const unsigned int timestamp, std::vector<Event> &output);

/**
 * @brief Base class for event simulators.
 */
class EventSimulator {
public:
  /**
   * @brief Returns the simulated events from the simulator
   *
   * @param prev_frame Frame form the previous time step
   * @param frame Frame form the current time step
   * @param prev_timestamp Time stamp of the previous time step
   * @param timestamp Time stamp of the current time step
   * @param num_frames Number of frames
   */
  virtual std::vector<Event>& getEvents(const cv::Mat prev_frame, const cv::Mat frame,
                                        const unsigned int prev_timestamp,
                                        const unsigned int timestamp,
                                        int& num_frames) = 0;

  /**
   * @brief Returns an accumulated event frame given the events
   *        from the simulator
   *
   * @param prev_frame Frame form the previous time step
   * @param frame Frame form the current time step
   * @param num_frames Number of frames
   */
  virtual std::vector<cv::Mat>& getEventFrame(const cv::Mat prev_frame, const cv::Mat frame,
                                              int& num_frames) = 0;

  /**
   * @brief Sets the frame size
   *
   * @param frame_size Size of the fame
   */
  virtual void setup(cv::Size frame_size) { frame_size_ = frame_size; }

  /**
   * @brief Returns the name of the event simulator
   */
  virtual std::string getName() = 0;

protected:
  /// Size of the frame
  cv::Size frame_size_;

  /// Vector containing the accumulated event frames
  std::vector<cv::Mat> out_frames_;

  /// Vector containing the events
  std::vector<Event> events_;
};
