#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <eigen_conversions/eigen_msg.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();  

  moveit::planning_interface::MoveGroupInterface move_group("tm_arm");

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup("tm_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  robot_state::RobotState start_state(*move_group.getCurrentState());

  start_state.clearAttachedBody("camera_link_optical");
  start_state.clearAttachedBody("camera_link");


  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 0.0104404;
  start_pose2.orientation.x = 0.70703;
  start_pose2.orientation.y = 0.70703;
  start_pose2.orientation.z = 0.0104404;
  start_pose2.position.x = 0.4175;
  start_pose2.position.y = -0.112007;
  start_pose2.position.z = 0.360155;
  std::vector<double> joint_values;
  ROS_INFO_NAMED("tutorial", "current joint values");
  start_state.copyJointGroupPositions(joint_model_group, joint_values);
  for(int i=0; i < 6; ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }


  //forward kenimatics
  const Eigen::Affine3d &end_effector_state = start_state.getGlobalLinkTransform("tip_link");

  // Print end-effector pose. Remember that this is in the model frame 
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  geometry_msgs::Pose end_effector_state_pose;

  tf::poseEigenToMsg(end_effector_state,end_effector_state_pose);

  std::cout<<"end_effector_state_pose are"<<std::endl;
  std::cout<<"x: "<<end_effector_state_pose.position.x<<",y: "<<end_effector_state_pose.position.y<<",z: "<<end_effector_state_pose.position.z<<std::endl;
  std::cout<<"x: "<<end_effector_state_pose.orientation.x<<",y: "<<end_effector_state_pose.orientation.y<<",z: "<<end_effector_state_pose.orientation.z
  <<",w: "<<end_effector_state_pose.orientation.w<<std::endl;

  bool isIkSuccess = start_state.setFromIK(joint_model_group, start_pose2);
  
  if(isIkSuccess){
    ROS_INFO_NAMED("tutorial", "ik success!!");
    start_state.copyJointGroupPositions(joint_model_group, joint_values);
    for(int i=0; i < 6; ++i)
    {
      ROS_INFO("Joint %d: %f", i, joint_values[i]);
    }
  }
  else{
    ROS_INFO_NAMED("tutorial", "ik fail!!");
  }
  ros::shutdown();
  return 0;
}