# Techman Robot Ros2 and Ros1 Bridge
:warning:
This is techman_robot_grasp_ros2 ros1 bridge, so suggest to use it with [techman_robot_grasp_ros2](https://github.com/JuFengWu/techman_robot_grasp_ros2)



### Main purpose of this project
In Ros2, Moveit is still in alpha version. The stable Moveit is still in Ros1.
So I create this project to launch Moveit and send the robot joint trajectory back to Ros2.

### Test Moveit functoin on Ros1 envirnoment
1. Build this project.
2. Open a terminal and type ``rosrun moveit_action_pkg get_trajectory``
3. Open another terminal and type ``roslaunch moveit_action_pkg tm_movit_bringup_test1.launch ``, the rviz, Moveit and joint_state_publisher will be launched.
4. Use the gui of joint_state_publisher to move robot to the place you want it to move.
5. Click the "plainning" button, you can see the "Plan and Execute" button.
6. Click "Plan and Execute" button, you can see the terminal 1 shows up the points which ready send back to Ros2.

### Test Moveit function without 3rd part apps
1. Build this project
2. Open a terminal and type ``roslaunch moveit_action_pkg tm_movit_bringup_test2.launch``
3. Open another terminal and type ``rosrun moveit_action_pkg set_command``.
4. You can see the first terminal show up the trajectory points.

### Use Ros2 simulator to send and get command
1. Build this project
2. Open a terminal and type ``roslaunch moveit_action_pkg tm_movit_bringup.launch``
3. Open another terminal and type ``rosrun ros2_send_command_test ros2_send_get_simulator``.
4. You can see the second terminal print up the trajectory result from the Moveit.
5. There are two trajectory. The first is send joint target and current joint position to get the trajevtories; the second is send cartesian target and current joint position to get the trajevtories.