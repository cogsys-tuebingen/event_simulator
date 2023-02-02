#include <event_simulator/Loader.h>

std::string VideoLoader::getFileName() {
  std::string str(path_);
  return str.substr(str.find_last_of("/\\") + 1);
}

void OpenCVLoader::load(const std::string path, const int height, const int width) {
  path_ = path;
  cv::VideoCapture cap(path);

  if (!cap.isOpened()) {
    throw std::runtime_error("Error reading video file. Does it exist?");
  }

  num_frames_ = cap.get(cv::CAP_PROP_FRAME_COUNT);
  frame_rate_ = cap.get(cv::CAP_PROP_FPS);

  if (height == 0) {
    res_y_ = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  } else {
    res_y_ = height;
  }
  if (width == 0) {
    res_x_ = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  } else {
    res_x_ = width;
  }

  std::cout << "Creating framebuffer of: " << num_frames_ << " n Frames"
            << std::endl;

  frame_buffer_.reserve(num_frames_);

  for (int i = 0; i < num_frames_; i++) {
    cv::Mat frame;
    cap >> frame;
    // Resize frame to chosen resolution if frame is bigger
    if (res_y_ > 0 && res_x_ > 0) {
      cv::Mat resized_frame;
      cv::resize(frame, resized_frame, cv::Size(res_x_, res_y_), cv::INTER_LINEAR);
      frame_buffer_.emplace_back(resized_frame);
    } else {
      frame_buffer_.emplace_back(frame);
    }
  }

  std::cout << "Finished creating framebuffer" << std::endl << std::endl;
}

void OpenCVLoader::release() {
  frame_buffer_.clear();
}
