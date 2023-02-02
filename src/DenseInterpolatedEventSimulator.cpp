#include <event_simulator/DenseInterpolatedEventSimulator.h>

DenseInterpolatedEventSimulator::DenseInterpolatedEventSimulator(
    const std::shared_ptr<DenseOpticalFlowCalculator> optical_flow_calculator,
    const int num_inter_frames, const int c_pos, const int c_neg)
    : num_inter_frames_{num_inter_frames},
      optical_flow_calculator_{optical_flow_calculator},
      name_{"DenseInterpolatedEventSimulator"}, c_pos_{c_pos}, c_neg_{c_neg} {}

std::vector<Event>& DenseInterpolatedEventSimulator::getEvents(
    cv::Mat prev_frame, cv::Mat frame, unsigned int prev_timestamp,
    unsigned int timestamp, int& num_frames) {
  num_frames = 1;

  cv::Mat flow = optical_flow_calculator_->calculateFlow(prev_frame, frame);
  cv::Mat prev_inter_frame = prev_frame;

  events_.clear();
  std::vector<cv::Mat> darker_frames;
  std::vector<cv::Mat> lighter_frames;
  for (int j = 0; j < num_inter_frames_ + 2; j++) {
    float alpha =
        static_cast<double>(j) / static_cast<double>(num_inter_frames_ + 1.0);
    unsigned int current_timestamp =
        prev_timestamp + alpha * (timestamp - prev_timestamp);
    cv::Mat interflow = alpha * flow;
    cv::Mat map(flow.size(), CV_32FC2);
    for (int y = 0; y < map.rows; ++y) {
      for (int x = 0; x < map.cols; ++x) {
        cv::Point2f f = interflow.at<cv::Point2f>(y, x);
        map.at<cv::Point2f>(y, x) = cv::Point2f(x + f.x, y + f.y);
      }
    }

    cv::Mat inter_frame;
    cv::Mat flow_parts[2];
    cv::split(map, flow_parts);
    cv::remap(prev_frame, inter_frame, flow_parts[0], flow_parts[1],
              cv::INTER_LINEAR);

    cv::Mat darker, lighter, darker_binary, lighter_binary;

    cv::subtract(prev_inter_frame, inter_frame, darker);
    cv::subtract(inter_frame, prev_inter_frame, lighter);

    cv::threshold(lighter, lighter, c_pos_, 255, cv::THRESH_BINARY);
    cv::threshold(darker, darker, c_neg_, 255, cv::THRESH_BINARY);

    packEvents(lighter, darker, current_timestamp, events_);

    prev_inter_frame = inter_frame;
  }

  return events_;
}

std::vector<cv::Mat>&
DenseInterpolatedEventSimulator::getEventFrame(const cv::Mat prev_frame,
                                               const cv::Mat frame, int& num_frames) {
  num_frames = 1;

  cv::Mat flow = optical_flow_calculator_->calculateFlow(prev_frame, frame);
  cv::Mat prev_inter_frame = prev_frame;

  cv::Mat darker_frame = cv::Mat::zeros(prev_frame.size(), CV_8UC1);
  cv::Mat lighter_frame = cv::Mat::zeros(prev_frame.size(), CV_8UC1);

  std::vector<cv::Mat> darker_frames;
  std::vector<cv::Mat> lighter_frames;
  for (int j = 0; j < num_inter_frames_ + 2; j++) {
    float alpha =
        static_cast<double>(j) / static_cast<double>(num_inter_frames_ + 1.0);
    cv::Mat interflow = alpha * flow;
    cv::Mat map(flow.size(), CV_32FC2);
    for (int y = 0; y < map.rows; ++y) {
      for (int x = 0; x < map.cols; ++x) {
        cv::Point2f f = interflow.at<cv::Point2f>(y, x);
        map.at<cv::Point2f>(y, x) = cv::Point2f(x + f.x, y + f.y);
      }
    }

    cv::Mat inter_frame;
    cv::Mat flow_parts[2];
    cv::split(map, flow_parts);
    cv::remap(prev_frame, inter_frame, flow_parts[0], flow_parts[1],
              cv::INTER_LINEAR);

    cv::Mat darker, lighter, darker_binary, lighter_binary;

    cv::subtract(prev_inter_frame, inter_frame, darker);
    cv::subtract(inter_frame, prev_inter_frame, lighter);

    cv::threshold(lighter, lighter, c_pos_, 255, cv::THRESH_BINARY);
    cv::threshold(darker, darker, c_neg_, 255, cv::THRESH_BINARY);

    cv::add(darker, darker_frame, darker_frame);
    cv::add(lighter, lighter_frame, lighter_frame);

    prev_inter_frame = inter_frame;
  }

  out_frames_.resize(num_frames);

  std::vector<cv::Mat> channels;
  cv::Mat zeros = cv::Mat::zeros(prev_frame.size(), CV_8UC1);
  channels.push_back(darker_frame);
  channels.push_back(zeros);
  channels.push_back(lighter_frame);
  cv::merge(channels, out_frames_.at(0));

  return out_frames_;
}
