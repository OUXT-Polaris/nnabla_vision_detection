cmake_minimum_required(VERSION 2.8.3)
project(nnabla_vision_detection)

add_compile_options(-std=c++11)

find_package(CUDA)

if(CUDA_FOUND)
  add_definitions(-DWITH_CUDA)
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  vision_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES nnabla_vision_detection
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs vision_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)