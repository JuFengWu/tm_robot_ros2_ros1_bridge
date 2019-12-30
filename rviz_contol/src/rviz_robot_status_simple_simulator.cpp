#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "../include/modle_interface.hpp"
#include <thread>
#include "boost/date_time/posix_time/posix_time.hpp"

class RvizRobotStatusSimulator
{
private:
  ros::Subscriber targetJointSub;
  ros::Publisher jointStatePub;
  ros::NodeHandle n;
  sensor_msgs::JointState targetJoint;

  std::unique_ptr<ModleInterface> robotModle;
    
  void target_joint(sensor_msgs::JointState targetJoint);
  void joint_state_publish();
public:
  static void wait_connect_success(ros::Publisher publisher);
  const static int jointNumber = 6;
  RvizRobotStatusSimulator();
};

RvizRobotStatusSimulator::RvizRobotStatusSimulator():robotModle(std::make_unique<TmGraspModel>())
{
  jointStatePub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
  RvizRobotStatusSimulator::wait_connect_success(jointStatePub);
  targetJointSub = n.subscribe("tm_joint_target", 1000, &RvizRobotStatusSimulator::target_joint,this);

  robotModle->set_model_name(targetJoint);
  
  robotModle->set_model_initial_pose(targetJoint,0);

  std::thread(&RvizRobotStatusSimulator::joint_state_publish,this).detach();
}

void RvizRobotStatusSimulator::joint_state_publish(){

  ros::Rate loopRate(100);

  while(ros::ok()){
    targetJoint.header.stamp = ros::Time::now();
    jointStatePub.publish(targetJoint);
    ros::spinOnce();

    loopRate.sleep();
  }
}

void RvizRobotStatusSimulator::target_joint(sensor_msgs::JointState newTargetJoint){
  
  if(newTargetJoint.position.size()<jointNumber){
    std::cout<<"new target error"<<std::endl;
    return;
  }
  for(int i=0;i<jointNumber;i++){
    targetJoint.position[i] = newTargetJoint.position[i];
  }
}

void RvizRobotStatusSimulator::wait_connect_success(ros::Publisher publisher){
  ros::Rate loopRate(10);//100ms
  while(publisher.getNumSubscribers()== 0){
    std::cout<<"wait connect success!"<<std::endl;
    loopRate.sleep();
  }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "rviz_control");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::unique_ptr<RvizRobotStatusSimulator> rvizRobotStatusSimulator = std::make_unique<RvizRobotStatusSimulator>();
   
  ros::waitForShutdown();
}
