#include <event_simulator/FarnebackFlowCalculator.h>

FarnebackFlowCalculator::FarnebackFlowCalculator()
    : name_{"FarnebackFlowCalculator"} {}

cv::Mat FarnebackFlowCalculator::calculateFlow(const cv::Mat prev_frame,
                                               const cv::Mat frame) {
  cv::Mat flow;
  cv::calcOpticalFlowFarneback(prev_frame, frame, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
  return flow;
}
