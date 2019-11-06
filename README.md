# Techman Robot Ros2 and Ros1 Bridge
:warning:
This is techman_robot_grasp_ros2 ros1 bridge, so suggest to use it with [techman_robot_grasp_ros2](https://github.com/JuFengWu/techman_robot_grasp_ros2)

:warning:
This project is under construct.

### Main purpose of this project
In Ros2, Moveit is still in alpha version. The stable Moveit is still in Ros1.
So I create this project to launch moveit and send the robot joint trajectory to ros2.

### Test Moveit functoin on Ros1 envirnoment
1. Build this project.
2. Open a terminal and type ``rosrun moveit_action_pkg get_trajectory``
3. Open another terminal and type ``roslaunch moveit_action_pkg tm_movit_bringup.launch ``, the rviz, moveit and joint_state_publisher will be launched.
4. Use the gui of joint_state_publisher to move robot to the place you want it to move.
5. Click the "plainning" button, you can see the "Plan and Execute" button.
6. Click "Plan and Execute" button, you can see the terminal 1 shows up the points which ready send back to Ros2.
