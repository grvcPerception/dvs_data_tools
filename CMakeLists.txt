cmake_minimum_required(VERSION 2.8.3)
project(dvs_data_tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native" )

## Find catkin macros and libraries
find_package(OpenCV 3.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dvs_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  message_generation
)

find_package(cmake_modules REQUIRED)

# search for all msg files
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

# notify catkin to look at the previously found msg files
#add_message_files(
#  FILES
#  ${messages_to_build}
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  dvs_msgs
#)

catkin_package(
 CATKIN_DEPENDS dvs_msgs geometry_msgs roscpp rospy std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(eventDataTools
  src/events2frames.cpp
  src/utils.cpp
)

add_executable(eventFrame_node src/dvs_events2frames_node.cpp)
add_executable(stereoTest_node src/stereo_test.cpp)

target_link_libraries(eventDataTools ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
target_link_libraries(eventFrame_node eventDataTools ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
target_link_libraries(stereoTest_node eventDataTools ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
