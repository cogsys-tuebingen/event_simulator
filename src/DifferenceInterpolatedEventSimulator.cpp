#include <event_simulator/DifferenceInterpolatedEventSimulator.h>

DifferenceInterpolatedEventSimulator::DifferenceInterpolatedEventSimulator(
    const std::shared_ptr<SparseOpticalFlowCalculator> optical_flow_calculator,
    const int num_inter_frames, const int c_pos, const int c_neg,
    const int c_pos_inter, const int c_neg_inter)
    : num_inter_frames_{num_inter_frames},
      optical_flow_calculator_{optical_flow_calculator},
      name_{"DifferenceInterpolatedEventSimulator"},
      c_pos_{c_pos},
      c_neg_{c_neg},
      c_pos_inter_{c_pos_inter},
      c_neg_inter_{c_neg_inter} {}

void DifferenceInterpolatedEventSimulator::setup(const cv::Size frame_size) {
  frame_size_ = frame_size;
  std::cout << "Frame size of: " << frame_size << std::endl;
  darker_cached_mask_ = cv::Mat::zeros(frame_size, CV_8UC1);
  lighter_cached_mask_ = cv::Mat::zeros(frame_size, CV_8UC1);
}

std::vector<Event>& DifferenceInterpolatedEventSimulator::getEvents(
    const cv::Mat prev_frame, const cv::Mat frame,
    const unsigned int prev_timestamp, const unsigned int timestamp,
    int& num_frames) {
  num_frames = 1;
  const auto& size = prev_frame.size;
  const auto& cols = prev_frame.cols;
  const auto& rows = prev_frame.rows;
  const int type = prev_frame.type();
  cv::Mat darker(rows, cols, type);
  cv::Mat lighter(rows, cols, type);
  cv::Mat darker_mask(rows, cols, type);
  cv::Mat lighter_mask(rows, cols, type);
  cv::Mat lighter_events(rows, cols, type);
  cv::Mat darker_events(rows, cols, type);

  events_.clear();

  cv::subtract(prev_frame, frame, darker);
  cv::threshold(darker, darker_mask, c_pos_inter_, 255, cv::THRESH_TOZERO);

  cv::subtract(frame, prev_frame, lighter);
  cv::threshold(lighter, lighter_mask, c_neg_inter_, 255, cv::THRESH_TOZERO);

  cv::threshold(darker, darker_events, c_pos_, 255, cv::THRESH_BINARY);
  cv::threshold(lighter, lighter_events, c_neg_, 255, cv::THRESH_BINARY);

  // Option to also add events from the subtract of the current frame and the
  // previous frame
  /*
  std::vector<cv::Point2f> darker_points, lighter_points;
  cv::findNonZero(darker_events, darker_points);
  for (const auto& point : darker_points) {
    events_.emplace_back(point.x, point.y, prev_timestamp, false);
  }
  cv::findNonZero(lighter_events, lighter_points);
  for (const auto& point : lighter_points) {
    events_.emplace_back(point.x, point.y, prev_timestamp, true);
  }
  */

  std::vector<cv::Point2f> prev_darker_points, prev_lighter_points;
  cv::findNonZero(darker_cached_mask_, prev_darker_points);
  cv::findNonZero(lighter_cached_mask_, prev_lighter_points);

  std::vector<cv::Point2f> next_darker_points =
      optical_flow_calculator_->calculateFlow(darker_cached_mask_, darker_mask,
                                              prev_darker_points);
  std::vector<cv::Point2f> next_lighter_points =
      optical_flow_calculator_->calculateFlow(
          lighter_cached_mask_, lighter_mask, prev_lighter_points);

  int x_res = darker_mask.cols;
  int y_res = darker_mask.rows;

  for (int j = 0; j < num_inter_frames_; j++) {
    float alpha =
        static_cast<double>(j + 1) / static_cast<double>(num_inter_frames_ + 1);

    for (std::size_t i = 0; i < next_lighter_points.size(); i++) {
      cv::Point2f inter_point =
          prev_lighter_points[i] +
          (next_lighter_points[i] - prev_lighter_points[i]) * alpha;
      unsigned int current_timestamp =
          prev_timestamp + (timestamp - prev_timestamp) * alpha;
      // check if in bounds
      if (inter_point.x > (x_res - 1) || inter_point.y > (y_res - 1) ||
          inter_point.x < 0 || inter_point.y < 0) {
        continue;
      }
      events_.emplace_back(inter_point.x, inter_point.y, current_timestamp,
                           true);
    }

    for (std::size_t i = 0; i < next_darker_points.size(); i++) {
      cv::Point2f inter_point =
          prev_darker_points[i] +
          (next_darker_points[i] - prev_darker_points[i]) * alpha;
      unsigned int current_timestamp =
          prev_timestamp + (timestamp - prev_timestamp) * alpha;
      // check if in bounds
      if (inter_point.x > (x_res - 1) || inter_point.y > (y_res - 1) ||
          inter_point.x < 0 || inter_point.y < 0) {
        continue;
      }
      events_.emplace_back(inter_point.x, inter_point.y, current_timestamp,
                           false);
    }
  }

  darker_cached_mask_ = darker_mask;
  lighter_cached_mask_ = lighter_mask;

  return events_;
}

std::vector<cv::Mat>& DifferenceInterpolatedEventSimulator::getEventFrame(
    const cv::Mat prev_frame, const cv::Mat frame, int& num_frames) {
  num_frames = 1;
  cv::Mat darker, lighter, darker_mask, lighter_mask, lighter_events,
      darker_events;

  out_frames_.resize(num_frames);

  cv::subtract(prev_frame, frame, darker);
  cv::threshold(darker, darker_mask, c_pos_inter_, 255, cv::THRESH_TOZERO);

  cv::subtract(frame, prev_frame, lighter);
  cv::threshold(lighter, lighter_mask, c_neg_inter_, 255, cv::THRESH_TOZERO);

  // Option to also add events from the subtract of the current frame and the
  // previous frame
  //cv::threshold(darker, darker_events, c_pos_, 255, cv::THRESH_BINARY);
  //cv::threshold(lighter, lighter_events, c_neg_, 255, cv::THRESH_BINARY);

  std::vector<cv::Point2f> prev_darker_points, prev_lighter_points;
  cv::findNonZero(darker_cached_mask_, prev_darker_points);
  cv::findNonZero(lighter_cached_mask_, prev_lighter_points);

  std::vector<cv::Point2f> next_darker_points =
      optical_flow_calculator_->calculateFlow(darker_cached_mask_, darker_mask,
                                              prev_darker_points);
  std::vector<cv::Point2f> next_lighter_points =
      optical_flow_calculator_->calculateFlow(
          lighter_cached_mask_, lighter_mask, prev_lighter_points);

  int x_res = darker_mask.cols;
  int y_res = darker_mask.rows;
  cv::Mat time = cv::Mat::zeros(lighter_mask.size(), CV_8UC1);

  for (int j = 0; j < num_inter_frames_; j++) {
    float alpha =
        static_cast<double>(j + 1) / static_cast<double>(num_inter_frames_ + 1);

    for (uint i = 0; i < next_lighter_points.size(); i++) {
      cv::Point2f inter_point =
          prev_lighter_points[i] +
          (next_lighter_points[i] - prev_lighter_points[i]) * alpha;
      // check if in bounds
      if (inter_point.x > (x_res - 1) || inter_point.y > (y_res - 1) ||
          inter_point.x < 0 || inter_point.y < 0)
        continue;
      lighter_events.at<uchar>(inter_point) = 255;
    }

    for (uint i = 0; i < next_darker_points.size(); i++) {
      cv::Point2f inter_point =
          prev_darker_points[i] +
          (next_darker_points[i] - prev_darker_points[i]) * alpha;
      // check if in bounds
      if (inter_point.x > (x_res - 1) || inter_point.y > (y_res - 1) ||
          inter_point.x < 0 || inter_point.y < 0)
        continue;
      darker_events.at<uchar>(inter_point) = 255;
    }
  }

  darker_cached_mask_ = darker_mask;
  lighter_cached_mask_ = lighter_mask;

  std::vector<cv::Mat> channels;

  channels.push_back(lighter_events);
  channels.push_back(time);
  channels.push_back(darker_events);
  cv::merge(channels, out_frames_.at(0));
  return out_frames_;
}
