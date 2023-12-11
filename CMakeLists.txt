cmake_minimum_required(VERSION 3.10)
project(dist_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++2a -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  camera_apps_msgs
  message_filters
)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dist_test
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs cv_bridge dist_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(use_pixel src/use_pixel.cpp)

add_dependencies(use_pixel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(use_pixel ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
