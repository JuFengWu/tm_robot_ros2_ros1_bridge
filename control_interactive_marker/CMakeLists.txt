cmake_minimum_required(VERSION 2.8.3)
project(control_interactive_marker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  interactive_markers
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_action_pkg
#  CATKIN_DEPENDS actionlib 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_interactive_marker src/robot_control_interactive_marker.cpp)
target_link_libraries(control_interactive_marker ${catkin_LIBRARIES})
