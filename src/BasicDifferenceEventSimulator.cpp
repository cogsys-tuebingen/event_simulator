#include <event_simulator/BasicDifferenceEventSimulator.h>

BasicDifferenceEventSimulator::BasicDifferenceEventSimulator(const int c_pos,
                                                             const int c_neg)
    : c_pos_{c_pos}, c_neg_{c_neg}, name_{"BasicDifferenceEventSimulator"} {}

std::vector<Event>& BasicDifferenceEventSimulator::getEvents(
    const cv::Mat prev_frame, const cv::Mat frame, const unsigned int prev_timestamp,
    const unsigned int timestamp, int& num_frames) {
  num_frames = 1;

  cv::Mat darker, lighter, darker_binary, lighter_binary;

  subtract(prev_frame, frame, darker);
  cv::threshold(darker, darker_binary, c_pos_, 255, cv::THRESH_BINARY);

  subtract(frame, prev_frame, lighter);
  cv::threshold(lighter, lighter_binary, c_neg_, 255, cv::THRESH_BINARY);

  events_.clear();
  packEvents(lighter_binary, darker_binary, timestamp, events_);

  return events_;
}

std::vector<cv::Mat>&
BasicDifferenceEventSimulator::getEventFrame(const cv::Mat prev_frame, const cv::Mat frame,
                                             int& num_frames) {
  num_frames = 1;

  cv::Mat darker, lighter, darker_binary, lighter_binary;

  out_frames_.resize(num_frames);

  subtract(prev_frame, frame, darker);
  cv::threshold(darker, darker_binary, c_pos_, 255, cv::THRESH_BINARY);

  subtract(frame, prev_frame, lighter);
  cv::threshold(lighter, lighter_binary, c_neg_, 255, cv::THRESH_BINARY);

  cv::Mat zeros = cv::Mat::zeros(darker_binary.size(), CV_8UC1);

  std::vector<cv::Mat> channels;
  channels.push_back(lighter_binary);
  channels.push_back(zeros);
  channels.push_back(darker_binary);
  merge(channels, out_frames_.at(0));

  return out_frames_;
}
