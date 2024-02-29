cmake_minimum_required(VERSION 2.8.3)
project(x_view_core)

# Add support for C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
  endif()
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# If no build type is given, set the build type to "Release"
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

message(STATUS "Building XView in ${CMAKE_BUILD_TYPE} mode.")

if(CMAKE_BUILD_TYPE MATCHES Debug)
  add_definitions(-DX_VIEW_DEBUG)
endif()

# Flag used by XView to know the path of where it is located.
add_definitions(-DX_VIEW_ROOT_DIR=${CMAKE_CURRENT_SOURCE_DIR}/)
add_definitions(-DX_VIEW_LOG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/logs/)
add_definitions(-DX_VIEW_OUT_DIR=${CMAKE_CURRENT_SOURCE_DIR}/output/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
)


find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)


# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
    #### SOURCE FILES ####
    src/x_view.cc
    src/x_view_tools.cc
    src/x_view_locator.cc

    # Datasets
    src/datasets/abstract_dataset.cc
    src/datasets/synthia_dataset.cc
    src/datasets/airsim_dataset.cc
    src/datasets/streetview_dataset.cc

    # Descriptors
    src/features/abstract_descriptor.cc
    src/features/graph_descriptor.cc
    src/features/graph.cc

    # Landmarks
    src/landmarks/abstract_semantic_landmark.cc
    src/landmarks/semantic_landmark_factory.cc
    src/landmarks/graph_landmark/graph_landmark.cc
    src/landmarks/graph_landmark/blob.cc
    src/landmarks/graph_landmark/blob_extractor.cc
    src/landmarks/graph_landmark/graph_builder.cc
    src/landmarks/graph_landmark/graph_drawer.cc
    src/landmarks/graph_landmark/depth_projector.cc

    # Feature matchers
    src/matchers/graph_matcher/graph_matcher.cc
    src/matchers/graph_matcher/graph_merger.cc
    src/matchers/graph_matcher/random_walker.cc
    src/matchers/graph_matcher/similarity_plotter.cc
    src/matchers/graph_matcher/vertex_similarity.cc

    # Parameters
    src/parameters/parameters.cc

    # Timer
    src/timer/null_timer.cc
    src/timer/timer.cc
    src/timer/timer_printer.cc
)

# Add dependencies to the executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/x_view_node.cpp)

# Add cmake target dependencies of the executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGET
