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
  nodelet
  pluginlib
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

## Macro
if(WITH_CUDA)
  macro(nnabla_vision_detection_add_nodelet node_name nodelet_cppfile)
    set(NODE_NAME ${node_name})
    set(NODELET_NAME nnabla_vision_detection/${node_name})
    configure_file(src/node/standalone_nodelet_exec.cpp.in ${node_name}.cpp @ONLY)
    add_executable(${node_name}_exe ${node_name}.cpp)
    SET_TARGET_PROPERTIES(${node_name}_exe PROPERTIES OUTPUT_NAME ${node_name})
    target_link_libraries(${node_name}_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} nnabla nnabla_utils)
    list(APPEND _nnabla_vision_detection_nodelet_cppfiles ${nodelet_cppfile})
    list(APPEND _nnabla_vision_detection_nodelet_targets ${node_name}_exe)
  endmacro()
else()
  macro(nnabla_vision_detection_add_nodelet node_name nodelet_cppfile)
    set(NODE_NAME ${node_name})
    set(NODELET_NAME nnabla_vision_detection/${node_name})
    configure_file(src/node/standalone_nodelet_exec.cpp.in ${node_name}.cpp @ONLY)
    add_executable(${node_name}_exe ${node_name}.cpp)
    SET_TARGET_PROPERTIES(${node_name}_exe PROPERTIES OUTPUT_NAME ${node_name})
    target_link_libraries(${node_name}_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} nnabla nnabla_utils nnabla_cuda)
    list(APPEND _nnabla_vision_detection_nodelet_cppfiles ${nodelet_cppfile})
    list(APPEND _nnabla_vision_detection_nodelet_targets ${node_name}_exe)
  endmacro()
endif()

## Add Nodelet
nnabla_vision_detection_add_nodelet(nnabla_vision_detection src/nodelet/nnabla_vision_detection_nodelet.cpp)

add_library(${PROJECT_NAME} SHARED
  src/nodelet/nodelet.cpp
  ${_nnabla_vision_detection_nodelet_cppfiles}
)

if(WITH_CUDA)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} nnabla nnabla_utils)
else()
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} nnabla nnabla_utils nnabla_cuda)
endif()

## Install
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${_nnabla_vision_detection_nodelet_targets}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)