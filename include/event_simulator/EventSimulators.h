#pragma once

#include <event_simulator/EventSimulator.h>
#include <event_simulator/OpticalFlow.h>
#ifdef USE_CUDA
#include <event_simulator/CudaFarnebackFlowCalculator.h>
#include <event_simulator/CudaLKOpticalFlowCalculator.h>
#endif
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <typeinfo>
