#pragma once

#include <event_simulator/EventSimulators.h>
#include <event_simulator/Loader.h>

#include <memory>
#include <opencv2/opencv.hpp>

/**
 * @brief Abstract class defining the interface for video player classes.
 */
class VideoPlayer {
 public:
  /**
   * @brief Constructor initializing the event simulator, the ROI and the loader
   *
   * @param event_simulator The event simulator used by the video player
   * @param res_x X resolution
   * @param res_y Y resolution
   */
  VideoPlayer(std::shared_ptr<EventSimulator> event_simulator, int res_x,
              int res_y);

  /**
   * @brief Loads a video, sets up the event simulator and runs the simulation
   *        @p repeats number of times.
   *
   * @param path Path to the video
   * @param height Height of the video frames
   * @param width Width of the video frames
   * @param repeats Number of times the simulation should be run
   * @param event_statistics Flag indicating if event statistics should be
   * calculated
   * @param record_video Flag indicating if a video should be recorded
   */
  void simulate(const std::string path, const int height = 0,
                const int width = 0, const int repeats = 1,
                const bool event_statistics = false,
                const bool record_video = false);

  /**
   * @brief Run a timed simulation. No event statistics will be calculated
   *        and not output video recorded.
   *
   * @param path Path to the video
   * @param height Height of the video frames
   * @param width Width of the video frames
   * @param repeats Number of times the simulation should be run
   * @param num_frames Number of frames in the video
   * @return The average run time per frame [ms]
   */
  double simulateTimed(const std::string path, const int height = 0,
                       const int width = 0, const int repeats = 1,
                       const int num_frames = 0);

  /**
   * @brief Simulates the event for a single frame and saves the result
   *
   * @param path Path to the video
   * @param height Height of the video frames
   * @param width Width of the video frames
   * @param frame_index Index of the frame
   */
  void saveSingleFrame(const std::string path, const int height = 0,
                       const int width = 0, const int frame_index = 1);

  /**
   * @brief Set the region of interest (ROI)
   *
   * @param roi Region of interest
   */
  void setROI(const cv::Rect roi) { roi_ = roi; }

 protected:
  /**
   * @brief Returns the next frame from the video loader
   */
  cv::Mat getNextFrame();

  /**
   * @brief Runs the simulation @p repeats number of times. If @p num_frames is
   * not set it will be determined by the video loader. Event statistics of the
   * simulation are recorded if @p event_statistics is set. A video is recorded
   * if @p record_video is set.
   *
   * @param repeats Number of simulations to run
   * @param num_frames Number of frames the video has
   * @param event_statistics Flag indicating if event statistics should be
   * calculated
   * @param record_video Flag indicating if a video should be recorded
   */
  virtual void loopSimulation(const int repeats = 1, int num_frames = 0,
                              const bool event_statistics = false,
                              const bool record_video = false) = 0;

  /**
   * @brief Returns the frame size of the video.
   */
  cv::Size getFrameSize() { return loader_->getFrame(0).size(); }

  /**
   * @brief Sets up the event simulator.
   */
  void setupEventSimulator() { event_simulator_->setup(getFrameSize()); }

  /// Event simulator
  std::shared_ptr<EventSimulator> event_simulator_;

  /// Number of the current frame
  std::size_t current_frame_;

  /// Region of intererst
  cv::Rect roi_;

  /// Video loader
  std::shared_ptr<VideoLoader> loader_;
};

/**
 * @brief Video player using OpenCV.
 */
class OpenCVPlayer : public VideoPlayer {
 public:
  /**
   * @brief Constructor initializing the base class, the wait time and pause
   *        flag.
   *
   * @param event_simulator Event simulator
   * @param wait_time_ms Waiting time
   * @param res_x X resolution
   * @param res_y Y resolution
   */
  OpenCVPlayer(std::shared_ptr<EventSimulator> event_simulator,
               const int wait_time_ms, const int res_x = 0,
               const int res_y = 0);

  /**
   * @brief Sets the event simulator
   *
   * @param event_simulator Event simulator
   */
  void setEventSimulator(std::shared_ptr<EventSimulator> event_simulator) {
    event_simulator_ = event_simulator;
  }

 private:
  void loopSimulation(const int repeats = 1, int num_frames = 0,
                      const bool event_statistics = false,
                      const bool record_video = false) override;

  /// Wait time [ms]
  int wait_time_ms_;
};

/**
 * @brief Video streamer class
 */
class VideoStreamer {
 public:
  /**
   * @brief Constructor initializing the event simulaor and disabling the ROI.
   *
   * @param event_simulator Event simulator
   */
  VideoStreamer(std::shared_ptr<EventSimulator> event_simulator);

  /**
   * @brief Simulate events given a video stream source.
   *
   * @param source_index Index of the video stream source
   */
  void simulateFromStream(const int source_index);

 protected:
  /// Event simulator
  std::shared_ptr<EventSimulator> event_simulator_;

  /// Region of interest
  cv::Rect roi_;

  /// Video loader
  std::shared_ptr<VideoLoader> loader_;
};
