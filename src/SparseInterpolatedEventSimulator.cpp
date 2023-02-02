#include <event_simulator/SparseInterpolatedEventSimulator.h>

SparseInterpolatedEventSimulator::SparseInterpolatedEventSimulator(
    const std::shared_ptr<SparseOpticalFlowCalculator> optical_flow_calculator,
    const int num_inter_frames, const int c_pos, const int c_neg)
    : num_inter_frames_{num_inter_frames},
      optical_flow_calculator_{optical_flow_calculator},
      name_{"SparseInterpolatedEventSimulator"},
      c_pos_{c_pos},
      c_neg_{c_neg} {}

std::vector<cv::Mat>& SparseInterpolatedEventSimulator::getEventFrame(
    const cv::Mat prev_frame, const cv::Mat frame, int& num_frames) {
  num_frames = 1;
  cv::Mat mask;

  cv::absdiff(prev_frame, frame, mask);
  cv::threshold(mask, mask, c_pos_, 255, cv::THRESH_BINARY);

  std::vector<cv::Point2f> prev_points;
  cv::findNonZero(mask, prev_points);

  std::vector<cv::Point2f> next_points =
      optical_flow_calculator_->calculateFlow(prev_frame, frame, prev_points);

  cv::Mat prev_inter_frame = prev_frame;

  cv::Mat darker_frame = cv::Mat::zeros(prev_frame.size(), CV_8UC1);
  cv::Mat lighter_frame = cv::Mat::zeros(prev_frame.size(), CV_8UC1);
  int x_res = mask.cols;
  int y_res = mask.rows;

  for (int j = 0; j < num_inter_frames_ + 2; j++) {
    float alpha =
        static_cast<double>(j) / static_cast<double>(num_inter_frames_ + 1);
    cv::Mat inter_frame;
    prev_frame.copyTo(inter_frame);

    for (uint i = 0; i < next_points.size(); i++) {
      cv::Point2f inter_point =
          prev_points[i] + (next_points[i] - prev_points[i]) * alpha;
      // check if in bounds
      if (inter_point.x > (x_res - 1) || inter_point.y > (y_res - 1) ||
          inter_point.x < 0 || inter_point.y < 0) {
        continue;
      }
      inter_frame.at<uchar>(inter_point) = prev_frame.at<uchar>(prev_points[i]);
    }

    cv::Mat darker, lighter, darker_binary, lighter_binary;

    cv::subtract(prev_inter_frame, inter_frame, darker);
    cv::subtract(inter_frame, prev_inter_frame, lighter);

    cv::add(darker, darker_frame, darker_frame);
    cv::add(lighter, lighter_frame, lighter_frame);

    cv::threshold(lighter_frame, lighter_frame, c_pos_, 255, cv::THRESH_BINARY);
    cv::threshold(darker_frame, darker_frame, c_neg_, 255, cv::THRESH_BINARY);
    prev_inter_frame = inter_frame;
  }

  out_frames_.resize(num_frames);

  std::vector<cv::Mat> channels;
  cv::Mat zeros = cv::Mat::zeros(prev_frame.size(), CV_8UC1);
  channels.push_back(lighter_frame);
  channels.push_back(zeros);
  channels.push_back(darker_frame);
  merge(channels, out_frames_.at(0));

  return out_frames_;
}

std::vector<Event>& SparseInterpolatedEventSimulator::getEvents(
    const cv::Mat prev_frame, const cv::Mat frame,
    const unsigned int prev_timestamp, const unsigned int timestamp,
    int& num_frames) {
  num_frames = 1;
  cv::Mat mask;

  cv::absdiff(prev_frame, frame, mask);
  cv::threshold(mask, mask, c_pos_, 255, cv::THRESH_BINARY);

  std::vector<cv::Point2f> prev_points;
  cv::findNonZero(mask, prev_points);

  std::vector<cv::Point2f> next_points =
      optical_flow_calculator_->calculateFlow(prev_frame, frame, prev_points);

  cv::Mat prev_inter_frame = prev_frame;

  events_.clear();
  cv::Mat darker_frame = cv::Mat::zeros(prev_frame.size(), CV_8UC1);
  cv::Mat lighter_frame = cv::Mat::zeros(prev_frame.size(), CV_8UC1);
  int x_res = mask.cols;
  int y_res = mask.rows;

  for (int j = 0; j < num_inter_frames_ + 2; j++) {
    float alpha =
        static_cast<double>(j) / static_cast<double>(num_inter_frames_ + 1);
    unsigned int current_timestamp =
        prev_timestamp + alpha * (timestamp - prev_timestamp);
    cv::Mat inter_frame;
    prev_frame.copyTo(inter_frame);

    for (uint i = 0; i < next_points.size(); i++) {
      cv::Point2f inter_point =
          prev_points[i] + (next_points[i] - prev_points[i]) * alpha;
      // check if in bounds
      if (inter_point.x > (x_res - 1) || inter_point.y > (y_res - 1) ||
          inter_point.x < 0 || inter_point.y < 0) {
        continue;
      }
      inter_frame.at<uchar>(inter_point) = prev_frame.at<uchar>(prev_points[i]);
      // time.at<uchar>(inter_point) = (int) (alpha * 127.0);
    }

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
