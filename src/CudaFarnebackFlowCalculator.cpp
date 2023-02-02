#include <event_simulator/CudaFarnebackFlowCalculator.h>

#include <opencv2/cudaoptflow.hpp>

CudaFarnebackFlowCalculator::CudaFarnebackFlowCalculator()
    : name_{"CudaFarnebackFlowCalculator"} {}

cv::Mat CudaFarnebackFlowCalculator::calculateFlow(const cv::Mat prev_frame,
                                                   const  cv::Mat frame) {
  cv::cuda::GpuMat flow_gpu;

  cv::cuda::GpuMat frame_gpu(frame);
  cv::cuda::GpuMat prev_frame_gpu(prev_frame);

  cv::Ptr<cv::cuda::FarnebackOpticalFlow> ofc =
      cv::cuda::FarnebackOpticalFlow::create(3, 0.5, false, 15);
  ofc->calc(prev_frame_gpu, frame_gpu, flow_gpu);

  cv::Mat flow;
  flow_gpu.download(flow);
  return flow;
}
