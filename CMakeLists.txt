cmake_minimum_required (VERSION 3.10) 
project(event_simulator
  LANGUAGES CXX C
  VERSION 0.1.0) 

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CheckLanguage)
check_language(CUDA)

if(CMAKE_CUDA_COMPILER)
  message("Build with CUDA support")
endif()

find_package(
  OpenCV
  COMPONENTS imgproc highgui video
  REQUIRED)

if(CMAKE_CUDA_COMPILER)
find_package(
  OpenCV
  COMPONENTS imgproc highgui video cudaoptflow
  REQUIRED)
endif()

find_package(Eigen3 REQUIRED NO_MODULE) 

find_package(OpenMP REQUIRED)

add_subdirectory(libs)

add_subdirectory(src)
