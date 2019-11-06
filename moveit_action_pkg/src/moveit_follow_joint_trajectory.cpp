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
    ROS_INFO("initial JointTrajectoryAction");
    actionServer.start();
  }

  ~JointTrajectoryAction(void)
  {
  }

  void printTrajectoryJointName(trajectory_msgs::JointTrajectory trajectory){
    ROS_INFO("trajectory joint name are:");
    for (int i =0;i<trajectory.joint_names.size();i++){
      std::cout<<"the "<<i<<" name is "<<trajectory.joint_names[i]<<std::endl;
    }
    ROS_INFO("the points number are");
    std::cout<<trajectory.points.size()<<std::endl;

    for(int i=0;i<trajectory.points.size();i++){
      std::cout<<trajectory.points[i].positions[0]<<","<<trajectory.points[i].positions[1]<<","<<trajectory.points[i].positions[2]<<","<<
      trajectory.points[i].positions[3]<<","<<trajectory.points[i].positions[4]<<","<<trajectory.points[i].positions[5]<<std::endl;
    }

  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr msg)
  {
    ROS_INFO("exec action moveit server ");
    if(!(*msg).trajectory.points.empty()){
      currentTrajectory = (*msg).trajectory;
      printTrajectoryJointName(currentTrajectory);
    }
    else{
      ROS_ERROR("joint trajectory from moveit is null!");
    }
    if(actionServer.isActive()){
      actionServer.setSucceeded();
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

