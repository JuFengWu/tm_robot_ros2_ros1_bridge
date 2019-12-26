#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "inverse_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();
/*
  ros::NodeHandle node_handle;

  // Start a service client
  ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  ros::Publisher robot_state_publisher =
      node_handle.advertise<moveit_msgs::DisplayRobotState>("tutorial_robot_state", 1);

  while (!service_client.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }

  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response;
  service_request.ik_request.group_name = "tm_arm";
  service_request.ik_request.pose_stamped.header.frame_id = "tip_link";
  //service_request.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  service_request.ik_request.pose_stamped.pose.position.x = 0.419715;
  service_request.ik_request.pose_stamped.pose.position.y = -0.121409;
  service_request.ik_request.pose_stamped.pose.position.z = 0.350776;

  service_request.ik_request.pose_stamped.pose.orientation.x = 0.707088;
  service_request.ik_request.pose_stamped.pose.orientation.y = 0.707036;
  service_request.ik_request.pose_stamped.pose.orientation.z = -0.00510927;
  service_request.ik_request.pose_stamped.pose.orientation.w = -0.00999615;

  service_client.call(service_request, service_response);

  if(service_response.error_code.val == service_response.error_code.SUCCESS){
    std::cout<<"success calculate"<<std::endl;
    std::cout<<"joints are"<<std::endl;
    for(unsigned int i=0;i<service_response.solution.joint_state.position.size();i++){
      std::cout<<service_response.solution.joint_state.position[i]<<","<<std::endl;
    }
    std::cout<<std::endl;
  }
  else
  {
      std::cout<<"there is some trobule to calculate IK, error code is "<<service_response.error_code.val<<std::endl;
  }
  ros::shutdown();
  return 0;
  */

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "right_arm" of the PR2
  // robot.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  //const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("tm_arm");

  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("tm_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the right arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  joint_values[0] = 0.2;
  joint_values[1] = 0.4;
  joint_values[2] = 0.4;
  joint_values[3] = 0.8;
  joint_values[4] = 0.3;
  joint_values[5] = 0.1;
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the right arm outside its joint limit */
  
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  kinematic_state->setJointPositions("shoulder_1_joint",&joint_values[0]);
  kinematic_state->setJointPositions("shoulder_2_joint",&joint_values[1]);
  kinematic_state->setJointPositions("elbow_1_joint",&joint_values[2]);
  kinematic_state->setJointPositions("wrist_1_joint",&joint_values[3]);
  kinematic_state->setJointPositions("wrist_2_joint",&joint_values[4]);
  kinematic_state->setJointPositions("wrist_3_joint",&joint_values[5]);

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "r_wrist_roll_link" which is the most distal link in the
  // "right_arm" of the robot.
  kinematic_state->setToRandomPositions(joint_model_group);

  moveit::planning_interface::MoveGroupInterface move_group("tm_arm");
  robot_state::RobotState start_state(*move_group.getCurrentState());

  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tip_link");

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  geometry_msgs::Pose end_effector_state_pose;

  tf::poseEigenToMsg(end_effector_state,end_effector_state_pose);

  std::cout<<"end_effector_state_pose are";
  std::cout<<"x: "<<end_effector_state_pose.position.x<<",y: "<<end_effector_state_pose.position.y<<",z: "<<end_effector_state_pose.position.z<<std::endl;
  std::cout<<"x: "<<end_effector_state_pose.orientation.x<<",y: "<<end_effector_state_pose.orientation.y<<",z: "<<end_effector_state_pose.orientation.z
  <<",w: "<<end_effector_state_pose.orientation.w<<std::endl;


  geometry_msgs::Pose pose;

  pose.position.x = -0.44652;
  pose.position.y = -0.15139;
  pose.position.z = 0.48965;

  pose.orientation.x = -0.63796;
  pose.orientation.y = 0.727609;
  pose.orientation.z = -0.116881;
  pose.orientation.w = -0.223453;

  Eigen::Affine3d mat;
  tf::poseMsgToEigen(pose, mat);

  ROS_INFO_STREAM("Translation: " << mat.translation());
  ROS_INFO_STREAM("Rotation: " << mat.rotation());

  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the right arm of the
  // PR2 robot. To solve IK, we will need the following:
  // * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain): end_effector_state that we computed in the step above.
  // * The number of attempts to be made at solving IK: 5
  // * The timeout for each attempt: 0.1 s

  kinematics::KinematicsQueryOptions o;
  o.return_approximate_solution = true;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }

  found_ik = start_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    start_state.copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
    start_state.copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }

  ROS_INFO("mat IK solution");

  double j1 = -0.05;
  double j2 = -0.06;
  double j3 = -1.19;
  double j4 = -0.15;
  double j5 = -1.09;
  double j6 = 0.12;

  //kinematic_state->setJointPositions("shoulder_1_joint",&j1);
  //kinematic_state->setJointPositions("shoulder_2_joint",&j2);
  //kinematic_state->setJointPositions("elbow_1_joint",&j3);
  //kinematic_state->setJointPositions("wrist_1_joint",&j4);
  //kinematic_state->setJointPositions("wrist_2_joint",&j5);
  //kinematic_state->setJointPositions("wrist_3_joint",&j6);
  
  


  found_ik = start_state.setFromIK(joint_model_group, mat, 10, 0.1);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }

  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the :moveit_core:`RobotState`.
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position,
                               jacobian);
  ROS_INFO_STREAM("Jacobian: " << jacobian);
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}