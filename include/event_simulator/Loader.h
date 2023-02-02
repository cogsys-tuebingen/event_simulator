#pragma once

#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Abstract class defining the interface for video loader classes.
 */
class VideoLoader {
public:
  /**
   * @brief Returns the file name of the video
   *
   * @return The file name of the video */
  std::string getFileName();

  /**
   * @brief Loads a video
   *
   * @param path Path to the video file
   * @param height Height of the video
   * @param width Width of the video
   */
  virtual void load(const std::string path, const int height = 0, const int width = 0) = 0;

  /**
   * @brief Release video resources
   */
  virtual void release() = 0;

  /**
   * @brief Returns the number of frames in the video
   *
   * @return Number of frames
   */
  std::size_t getNumFrames() { return num_frames_; }

  /**
   * @brief Returns the frame rate of the video
   *
   * @return the frame rate of the video
   */
  virtual int getFrameRate() = 0;

  /**
   * @brief Returns the height of the video
   *
   * @return The height of the video
   */
  virtual int getFrameHeight() = 0;

  /**
   * @brief Returns the width of the video
   *
   * @return The width of the video
   */
  virtual int getFrameWidth() = 0;

  /**
   * @brief Returns frame number @p index of the video
   *
   * @param index The frame index
   */
  cv::Mat getFrame(const int index) const { return frame_buffer_[index]; }

protected:
  /// Frame buffer as vector of cv::Mat
  std::vector<cv::Mat> frame_buffer_;

  /// Number of frames of the video
  std::size_t num_frames_;

  /// Frame rate of the video
  int frame_rate_;

  /// Path to the video file
  std::string path_;

  /// X resolution
  int res_x_;

  /// Y resolution
  int res_y_;
};

/**
 * @brief Class to load videos with OpenCV
 */
class OpenCVLoader : public VideoLoader {
public:
  /**
   * @brief Loads a video
   *
   * @param path Path to the video file
   * @param height Height of the video
   * @param width Width of the video
   */
  void load(const std::string path, const int height = 0, const int width = 0) override;

  /**
   * @brief Clears the frame buffer
   */
  void release() override;

  int getFrameRate(void) override { return frame_rate_; }

  int getFrameHeight() override { return res_y_; }

  int getFrameWidth() override { return res_x_; }
};
