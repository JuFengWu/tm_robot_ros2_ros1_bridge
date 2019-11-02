#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <iostream>
class JointTrajectoryAction
{
protected:
  ros::NodeHandle rosNode;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> actionServer;
  trajectory_msgs::JointTrajectory currentTrajectory;

  control_msgs::FollowJointTrajectoryResult result;  

public:

  JointTrajectoryAction(std::string name) :
    actionServer(rosNode, name, boost::bind(&JointTrajectoryAction::executeCB, this, _1), false)
  {
    actionServer.start();
  }

  ~JointTrajectoryAction(void)
  {
  }

  void printTrajectoryJointName(trajectory_msgs::JointTrajectory trajectory){
    std::cout<<"trajectory joint name are:"<<std::endl;
    for (int i =0;i<trajectory.joint_names.size();i++){
      std::cout<<"the "<<i<<" name is "<<trajectory.joint_names[i]<<std::endl;
    }
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr msg)
  {
    std::cout<<"exec action moveit server "<<std::endl;
    if(!(*msg).trajectory.points.empty()){
      currentTrajectory = (*msg).trajectory;
      printTrajectoryJointName(currentTrajectory);
    }
    else{
      ROS_ERROR("joint trajectory from moveit is null!");
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tm_joint_trajectory_action");

  JointTrajectoryAction jointTrajectoryAction("tm_arm_controller/follow_joint_trajectory");
  ros::spin();

  return 0;
}

