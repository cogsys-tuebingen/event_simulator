add_library(of_dis_c SHARED
  OF_DIS/FDF1.0.1/image.c
  OF_DIS/FDF1.0.1/opticalflow_aux.c
  OF_DIS/FDF1.0.1/solver.c)

add_library(of_dis_c::of_dis_c ALIAS of_dis_c)

target_include_directories(
  of_dis_c
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/libs/OF_DIS>)

target_compile_definitions(of_dis_c PUBLIC SELECTCHANNEL=1)

add_library(of_dis SHARED
  #OF_DIS/run_dense.cpp
  OF_DIS/oflow.cpp
  OF_DIS/patchgrid.cpp
  OF_DIS/refine_variational.cpp
  OF_DIS/patch.cpp)

add_library(of_dis::of_dis ALIAS of_dis)

target_link_libraries(of_dis PUBLIC opencv_core Eigen3::Eigen OpenMP::OpenMP_CXX of_dis_c)

target_include_directories(
  of_dis
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/libs/OF_DIS>)

target_compile_definitions(of_dis PUBLIC USE_PARALLEL_ON_FLOWAGGR SELECTMODE=1 SELECTCHANNEL=1 )

target_compile_options(
  of_dis
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

include(GNUInstallDirs)

install(
  TARGETS of_dis_c
  EXPORT of_dis_c-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
  TARGETS of_dis
  EXPORT of_dis-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
  EXPORT of_dis_c-targets
  FILE of_dis_c-targets.cmake
  NAMESPACE of_dis_c::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/of_dis_c)

install(
  EXPORT of_dis-targets
  FILE of_dis-targets.cmake
  NAMESPACE of_dis::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/of_dis)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake/of_dis_c-config.cmake.in
  ${CMAKE_BINARY_DIR}/cmake/of_dis_c-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/of_dis_c)

configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake/of_dis-config.cmake.in
  ${CMAKE_BINARY_DIR}/cmake/of_dis-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/of_dis)

write_basic_package_version_file(
  ${CMAKE_BINARY_DIR}/cmake/of_dis_c-config-version.cmake
  VERSION ${EVENT_SIMULATOR_VERSION}
  COMPATIBILITY AnyNewerVersion)

write_basic_package_version_file(
  ${CMAKE_BINARY_DIR}/cmake/of_dis-config-version.cmake
  VERSION ${EVENT_SIMULATOR_VERSION}
  COMPATIBILITY AnyNewerVersion)

export(
  EXPORT of_dis_c-targets
  FILE ${CMAKE_BINARY_DIR}/cmake/of_dis_c-targets.cmake
  NAMESPACE of_dis_c::)

export(
  EXPORT of_dis-targets
  FILE ${CMAKE_BINARY_DIR}/cmake/of_dis-targets.cmake
  NAMESPACE of_dis::)

install(FILES ${CMAKE_BINARY_DIR}/cmake/of_dis_c-config.cmake
              ${CMAKE_BINARY_DIR}/cmake/of_dis_c-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/of_dis_c)

install(FILES ${CMAKE_BINARY_DIR}/cmake/of_dis-config.cmake
              ${CMAKE_BINARY_DIR}/cmake/of_dis-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/of_dis)
