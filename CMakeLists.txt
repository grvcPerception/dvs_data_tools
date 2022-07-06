cmake_minimum_required(VERSION 2.8.3)
project(dvs_data_tools)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# Make the executable
cs_add_executable(eventFrame_node
  src/utils.cpp
  src/events2frames.cpp
	src/dvs_events2frames_node.cpp
)
# Link libraries
target_link_libraries(eventFrame_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_add_executable(stereoTest_node src/stereo_test.cpp)
target_link_libraries(stereoTest_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()
cs_export()
