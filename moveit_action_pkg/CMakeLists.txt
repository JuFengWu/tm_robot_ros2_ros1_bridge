cmake_minimum_required(VERSION 2.8.3)
project(moveit_action_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  moveit_visual_tools
  geometry_msgs
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

add_executable(set_command src/test_moveit_interaction.cpp src/moveit_interaction.cpp)
target_link_libraries(set_command ${catkin_LIBRARIES})

add_executable(pass_moveit src/pass_ros_command_to_moveit.cpp src/moveit_interaction.cpp)
target_link_libraries(pass_moveit ${catkin_LIBRARIES})