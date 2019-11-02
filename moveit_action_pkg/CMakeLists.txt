cmake_minimum_required(VERSION 2.8.3)
project(moveit_action_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_action_pkg
  CATKIN_DEPENDS actionlib moveit_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_trajectory src/moveit_follow_joint_trajectory.cpp)
target_link_libraries(get_trajectory ${catkin_LIBRARIES})

