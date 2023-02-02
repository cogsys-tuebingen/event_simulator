#include <event_simulator/LKOpticalFlowCalculator.h>

LKOpticalFlowCalculator::LKOpticalFlowCalculator(const int max_level,
                                                 const int window_size)
    : max_level_{max_level},
      window_size_{window_size}, name_{"LKOpticalFlowCalculator"} {}

std::vector<cv::Point2f>
LKOpticalFlowCalculator::calculateFlow(cv::Mat prev_frame, cv::Mat frame,
                                       std::vector<cv::Point2f> points) {
  if (points.size() == 0) {
    return std::vector<cv::Point2f>();
  }
  std::vector<uchar> status;
  std::vector<float> err;
  std::vector<cv::Point2f> new_points;

  cv::TermCriteria criteria = cv::TermCriteria(
      (cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);

  cv::calcOpticalFlowPyrLK(prev_frame, frame, points, new_points, status, err,
                           cv::Size(window_size_, window_size_), max_level_,
                           criteria);

  return new_points;
}
