#include <event_simulator/CudaLKOpticalFlowCalculator.h>

#include <opencv2/cudaoptflow.hpp>

CudaLKOpticalFlowCalculator::CudaLKOpticalFlowCalculator(const int max_lvl,
                                                         const int window_size)
    : max_lvl_{max_lvl},
      window_size_{window_size},
      name_{"CudaLKOpticalFlowCalculator"} {}

std::vector<cv::Point2f> CudaLKOpticalFlowCalculator::calculateFlow(
    const cv::Mat prev_frame, const cv::Mat frame,
    const std::vector<cv::Point2f> points) {
  if (points.size() == 0) {
    return std::vector<cv::Point2f>();
  }

  cv::cuda::GpuMat frame_gpu(frame);
  cv::cuda::GpuMat prev_frame_gpu(prev_frame);
  cv::cuda::GpuMat points_gpu(points);
  cv::cuda::GpuMat new_points_gpu, status_gpu;

  cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> ofc =
      cv::cuda::SparsePyrLKOpticalFlow::create(
          cv::Size(window_size_, window_size_), max_lvl_);
  ofc->calc(prev_frame_gpu, frame_gpu, points_gpu, new_points_gpu, status_gpu);

  std::vector<cv::Point2f> new_points(new_points_gpu.cols);
  new_points_gpu.download(new_points);

  return new_points;
}
