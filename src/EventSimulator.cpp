#include <event_simulator/EventSimulator.h>

void packEvents(const cv::Mat &lighter_events, const cv::Mat &darker_events,
                const unsigned int timestamp, std::vector<Event> &output) {
  std::vector<cv::Point2d> neg_polarity_events;
  std::vector<cv::Point2d> pos_polarity_events;

  cv::findNonZero(lighter_events, pos_polarity_events);
  cv::findNonZero(darker_events, neg_polarity_events);

  output.reserve(output.size() + neg_polarity_events.size() +
                 pos_polarity_events.size());

  for (const auto &point : neg_polarity_events) {
    output.emplace_back(point.x, point.y, timestamp, false);
  }
  for (const auto &point : pos_polarity_events) {
    output.emplace_back(point.x, point.y, timestamp, true);
  }
}
