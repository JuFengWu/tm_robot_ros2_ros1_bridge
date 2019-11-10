cmake_minimum_required(VERSION 2.8.3)
project(ros2_send_command_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_action_pkg
  CATKIN_DEPENDS actionlib geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_ros2_send_get src/test_ros2_send_get.cpp)
target_link_libraries(test_ros2_send_get ${catkin_LIBRARIES})
