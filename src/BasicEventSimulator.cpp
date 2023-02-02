#include <event_simulator/BasicEventSimulator.h>

BasicEventSimulator::BasicEventSimulator() : name_{"BasicRenderer"} {}

std::vector<Event>& BasicEventSimulator::getEvents(const cv::Mat prev_frame,
                                                   const cv::Mat frame,
                                                   const unsigned int prev_timestamp,
                                                   const unsigned int timestamp,
                                                   int& num_frames) {
  events_.clear();
  return events_;
}

std::vector<cv::Mat>& BasicEventSimulator::getEventFrame(const cv::Mat prev_frame,
                                                         const cv::Mat /*frame*/,
                                                         int& num_frames) {
  num_frames = 1;
  out_frames_.resize(1);
  out_frames_.at(0) = prev_frame;
  return out_frames_;
}
