#include <event_simulator/Player.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

/**
 * @brief Color to gray converstion
 *
 * @param frame Color frame in BGR
 */
cv::Mat toGray(const cv::Mat& frame) {
  cv::Mat grey_frame;
  cv::cvtColor(frame, grey_frame, cv::COLOR_BGR2GRAY);
  return grey_frame;
}

/**
 * @brief Returns a random hex number as string.
 *
 * @param length Lenght of the hexadecimal number
 */
std::string getRandomHex(int length) {
  std::array<char, 16> hexChar = {'0', '1', '2', '3', '4', '5', '6', '7',
                                  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

  std::string output;
  // Loop to print N integers
  for (int i = 0; i < length; i++) {
    output += hexChar[rand() % 16];
  }
  return output;
}

VideoPlayer::VideoPlayer(std::shared_ptr<EventSimulator> event_simulator,
                         int res_x, int res_y)
    : event_simulator_{event_simulator},
      current_frame_{0},
      roi_{cv::Rect(0, 0, 0, 0)},
      loader_{std::make_shared<OpenCVLoader>()} {}

void VideoPlayer::simulate(const std::string path, const int height,

                           const int width, const int repeats,
                           const bool event_statistics,
                           const bool record_video) {
  loader_->load(path, height, width);

  setupEventSimulator();

  int start = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();

  loopSimulation(repeats, 0, event_statistics, record_video);

  int end = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();

  loader_->release();

  int ms = end - start;
  std::cout << std::endl
            << "Frame time of "
            << static_cast<double>(ms) / static_cast<double>(current_frame_)
            << "ms" << std::endl;
  std::cout << "Diff of " << current_frame_ << " Frames in " << ms << "ms = "
            << static_cast<double>(current_frame_) /
                   (static_cast<double>(ms) / 1000.0)
            << " FPS" << std::endl;

  // Reset counter to be ready to play again
  current_frame_ = 0;
}

double VideoPlayer::simulateTimed(const std::string path, const int height,
                                  const int width, const int repeats,
                                  const int num_frames) {
  loader_->load(path, height, width);

  setupEventSimulator();

  int start = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();

  loopSimulation(repeats, num_frames);

  int end = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();

  loader_->release();

  int ms = (end - start);
  double frametime =
      static_cast<double>(ms) / static_cast<double>(current_frame_ * repeats);
  std::cout << std::endl
            << "Frame time of "
            << static_cast<double>(ms) / static_cast<double>(current_frame_)
            << "ms" << std::endl;
  std::cout << "Diff of " << current_frame_ << " Frames in " << ms << "ms = "
            << static_cast<double>(current_frame_) /
                   (static_cast<double>(ms) / 1000.0)
            << " FPS" << std::endl;

  // Reset counter to be ready to play again
  current_frame_ = 0;

  return frametime;
}

void VideoPlayer::saveSingleFrame(const std::string path, const int height,
                                  const int width, const int frame_index) {
  if (loader_->getNumFrames() < 3 || frame_index < 3) {
    std::cout << "ERROR: Frame index must be at least 3 and video must contain "
                 "3 frames"
              << std::endl;
  }

  loader_->load(path, height, width);

  setupEventSimulator();

  cv::Mat first = toGray(loader_->getFrame(frame_index - 2));
  cv::Mat second = toGray(loader_->getFrame(frame_index - 1));
  int num_frames;
  event_simulator_->getEventFrame(first, second, num_frames);
  auto results = event_simulator_->getEventFrame(
      second, toGray(loader_->getFrame(frame_index)), num_frames);

  std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
  std::string::size_type const p(base_filename.find_last_of('.'));
  std::string file_without_extension = base_filename.substr(0, p);
  std::string output_path = file_without_extension;

  for (int i = 0; i < num_frames; i++) {
    std::string random_id = getRandomHex(6);
    std::string file_name = std::to_string(frame_index) + "_" +
                            event_simulator_->getName() + ".png";
    if (roi_.width > 0 && roi_.height > 0) {
      cv::imwrite(output_path + "_" + file_name, results.at(i)(roi_));
    } else {
      cv::imwrite(output_path + "_" + file_name, results.at(i));
    }
    std::cout << "Saved frame " << frame_index << " to "
              << output_path + "_" + file_name << std::endl;
  }
}

cv::Mat VideoPlayer::getNextFrame() {
  current_frame_++;
  if (loader_->getNumFrames() == 0) {
    throw std::runtime_error(
        "No frames in video! Did you forget to call loader->load(path)?");
  }
  return loader_->getFrame(current_frame_ % loader_->getNumFrames());  //);
}

OpenCVPlayer::OpenCVPlayer(std::shared_ptr<EventSimulator> event_simulator,
                           const int wait_time_ms, const int res_x,
                           const int res_y)
    : VideoPlayer(event_simulator, res_x, res_y), wait_time_ms_{wait_time_ms} {}

void OpenCVPlayer::loopSimulation(const int repeats, int num_frames,
                                  const bool event_statistics,
                                  const bool record_video) {
  auto frame_rate = loader_->getFrameRate();
  double time_per_frame = 1.0 / static_cast<double>(frame_rate);
  if (num_frames == 0) {
    num_frames = loader_->getNumFrames();
  }
  double seconds = num_frames * time_per_frame;
  cv::Mat frame;
  cv::Mat prev_frame = toGray(getNextFrame());

  auto rows = loader_->getFrameHeight();
  auto cols = loader_->getFrameWidth();
  cv::Mat total_events_per_pixel = cv::Mat::zeros(rows, cols, CV_64F);
  cv::Mat pos_events_per_pixel = cv::Mat::zeros(rows, cols, CV_64F);
  cv::Mat neg_events_per_pixel = cv::Mat::zeros(rows, cols, CV_64F);

  std::cout << "Height: " << rows << ", width: " << cols << std::endl;
  std::cout << "Framerate: " << frame_rate << std::endl;
  std::cout << "Number of frames: " << num_frames << std::endl;
  std::cout << "Time per frame: " << time_per_frame << "s" << std::endl;
  std::cout << "Video duration: " << num_frames / frame_rate << "s"
            << std::endl;
  std::cout << "Waiting each frame for " << wait_time_ms_ << "ms" << std::endl;

  cv::VideoWriter video_capture;
  if (record_video) {
    const auto simulator_name = event_simulator_->getName();
    auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    video_capture.open(simulator_name + "_video.mp4", fourcc, 20 /*fps*/,
                       cv::Size(cols, rows));
  }

  bool should_quit = false;
  while (!should_quit) {
    if ((repeats > 1 && repeats <= (current_frame_ / num_frames)) ||
        (repeats == 1 && current_frame_ >= num_frames)) {
      break;
    }

    frame = toGray(getNextFrame());
    int num_frames;
    auto out_frames =
        event_simulator_->getEventFrame(prev_frame, frame, num_frames);

    if (record_video && video_capture.isOpened()) {
      for (const auto& frame : out_frames) {
        video_capture << frame;
      }
    }

    if (event_statistics) {
      double prev_timestamp = (current_frame_ - 1) * time_per_frame;
      double timestamp = current_frame_ * time_per_frame;
      auto events = event_simulator_->getEvents(
          prev_frame, frame, prev_timestamp, timestamp, num_frames);

      for (const auto& event : events) {
        total_events_per_pixel.at<double>(event.y, event.x) += 1.0;

        if (event.polarity) {
          pos_events_per_pixel.at<double>(event.y, event.x) += 1.0;
        } else {
          neg_events_per_pixel.at<double>(event.y, event.x) += 1.0;
        }
      }
    }

    if (wait_time_ms_ > 0) {
      for (int i = 0; i < num_frames; i++) {
        cv::imshow("OpenCVPlayer", out_frames.at(i));
        char c = static_cast<char>(cv::waitKey(wait_time_ms_));
        if (c == 27) {  // esc. to quit
          should_quit = true;
        }
        if (c == 115) {  // s for save
          std::stringstream ss;
          ss << "../res/export_frames/" << loader_->getFileName() << "_" << i
             << ".png";
          std::cout << "Saving frame to file: " << ss.str();
          cv::imwrite(ss.str(), out_frames.at(i));
        }
      }
      if (should_quit) {
        break;
      }
    }
    prev_frame = frame;
  }
  cv::destroyAllWindows();

  if (event_statistics) {
    std::ofstream myfile;
    const auto simulator_name = event_simulator_->getName();
    myfile.open(simulator_name + "_events_per_pixel.csv");
    myfile << "y,x,events_per_pixel,events_per_pixel_per_second\n";
    for (std::size_t y = 0; y < total_events_per_pixel.rows; y++) {
      for (std::size_t x = 0; x < total_events_per_pixel.cols; x++) {
        myfile << y << "," << x << ","
               << total_events_per_pixel.at<double>(y, x) << ","
               << total_events_per_pixel.at<double>(y, x) / seconds << '\n';
      }
    }
    myfile.close();

    double min, max;
    cv::minMaxIdx(total_events_per_pixel, &min, &max);
    std::cout << "Min: " << min << ", max: " << max << std::endl;
    cv::Mat output;
    cv::normalize(total_events_per_pixel, output, 0.0, 255.0, cv::NORM_MINMAX,
                  CV_8U);
    // events_per_pixel.convertTo(output, CV_8U, 1.0);

    cv::imwrite(simulator_name + "events_per_pixel_from_sim.png", output);
    cv::FileStorage fs(simulator_name + "events_per_pixel_from_sim.json",
                       cv::FileStorage::WRITE);
    fs << "total_events_per_pixel" << total_events_per_pixel;
    fs << "pos_events_per_pixel" << pos_events_per_pixel;
    fs << "neg_events_per_pixel" << neg_events_per_pixel;

    cv::Mat total_events_per_pixel_per_second;
    cv::Mat pos_events_per_pixel_per_second;
    cv::Mat neg_events_per_pixel_per_second;
    cv::multiply(cv::Mat::ones(rows, cols, CV_64F), total_events_per_pixel,
                 total_events_per_pixel_per_second, 1 / seconds);
    cv::multiply(cv::Mat::ones(rows, cols, CV_64F), pos_events_per_pixel,
                 pos_events_per_pixel_per_second, 1 / seconds);
    cv::multiply(cv::Mat::ones(rows, cols, CV_64F), neg_events_per_pixel,
                 neg_events_per_pixel_per_second, 1 / seconds);

    cv::minMaxIdx(total_events_per_pixel_per_second, &min, &max);
    std::cout << "Min: " << min << ", max: " << max << std::endl;
    cv::normalize(total_events_per_pixel_per_second, output, 0.0, 255.0,
                  cv::NORM_MINMAX, CV_8U);

    cv::imwrite(simulator_name + "events_per_pixel_per_second_from_sim.png",
                output);
    fs << "total_events_per_pixel_per_second"
       << total_events_per_pixel_per_second;
    fs << "pos_events_per_pixel_per_second" << pos_events_per_pixel_per_second;
    fs << "neg_events_per_pixel_per_second" << neg_events_per_pixel_per_second;
    fs.release();
  }
}

VideoStreamer::VideoStreamer(std::shared_ptr<EventSimulator> event_simulator)
    : event_simulator_{event_simulator}, roi_{cv::Rect(0, 0, 0, 0)} {}

void VideoStreamer::simulateFromStream(const int source_index) {
  cv::VideoCapture cap(source_index);
  if (!cap.isOpened()) {
    throw std::runtime_error("Error opening video source. Does it exist?");
  }

  cv::Mat frame, prev_frame;
  cap.read(prev_frame);
  prev_frame = toGray(prev_frame);

  event_simulator_->setup(prev_frame.size());

  while (true) {
    cap >> frame;
    frame = toGray(frame);
    int num_frames;
    auto out_frames =
        event_simulator_->getEventFrame(prev_frame, frame, num_frames);

    bool should_quit = false;
    for (int i = 0; i < num_frames; i++) {
      cv::imshow("OpenCVPlayer", out_frames.at(i));
      char c = static_cast<char>(cv::waitKey(1));
      if (c == 27) {  // esc. to quit
        should_quit = true;
      }
    }

    if (should_quit) {
      break;
    }

    prev_frame = frame;
  }
  cv::destroyAllWindows();
}
