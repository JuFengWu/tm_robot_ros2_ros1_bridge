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

add_executable(ros2_send_get_simulator src/ros2_send_get_simulator.cpp)
target_link_libraries(ros2_send_get_simulator ${catkin_LIBRARIES})
