set(${PROJECT_NAME}_SOURCES
  Loader.cpp
  Player.cpp
  OpticalFlow.cpp
  DenseOpticalFlowCalculator.cpp
  DISOpticalFlowCalculator.cpp
  SparseOpticalFlowCalculator.cpp
  LKOpticalFlowCalculator.cpp
  FarnebackFlowCalculator.cpp
  EventSimulator.cpp
  BasicDifferenceEventSimulator.cpp
  BasicEventSimulator.cpp
  DenseInterpolatedEventSimulator.cpp
  SparseInterpolatedEventSimulator.cpp
  DifferenceInterpolatedEventSimulator.cpp)

if(CMAKE_CUDA_COMPILER)
  list(APPEND ${PROJECT_NAME}_SOURCES CudaLKOpticalFlowCalculator.cpp CudaFarnebackFlowCalculator.cpp)
endif()

add_library(
  ${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

if(CMAKE_CUDA_COMPILER)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    opencv_imgproc
    opencv_highgui
    opencv_video
    opencv_cudaoptflow
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    of_dis)
else()
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    opencv_imgproc
    opencv_highgui
    opencv_video
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    of_dis)
endif()

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
         $<INSTALL_INTERFACE:include>
         ${catkin_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_compile_definitions(${PROJECT_NAME} PUBLIC USE_PARALLEL_ON_FLOWAGGR SELECTMODE=1 SELECTCHANNEL=1)

if(CMAKE_CUDA_COMPILER)
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_CUDA)
endif()

target_compile_options(
  ${PROJECT_NAME}
  PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
          -pipe
          -march=native
          -Wall
          -Wextra
          -msse4
          $<$<CONFIG:Release>:-O3>>
          $<$<CONFIG:Debug>:-Og
          -g
          -ggdb3
          >>
          $<$<CXX_COMPILER_ID:MSVC>:
          $<$<CONFIG:Debug>:/Od
          /Wall
          /Zi>>)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME})

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
             LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
             RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(GNUInstallDirs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/event_simulator
                  ${CMAKE_SOURCE_DIR}/libs/OF_DIS
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/event_simulator)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake/event_simulator-config.cmake.in
  ${CMAKE_BINARY_DIR}/cmake/event_simulator-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/event_simulator)

write_basic_package_version_file(
  ${CMAKE_BINARY_DIR}/cmake/event_simulator-config-version.cmake
  VERSION ${EVENT_SIMULATOR_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES ${CMAKE_BINARY_DIR}/cmake/event_simulator-config.cmake
              ${CMAKE_BINARY_DIR}/cmake/event_simulator-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/event_simulator)

export(
  EXPORT event_simulator-targets
  FILE ${CMAKE_BINARY_DIR}/cmake/event_simulator-targets.cmake
  NAMESPACE ${PROJECT_NAME}::)
