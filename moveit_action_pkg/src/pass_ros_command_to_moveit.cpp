#include "../include/moveit_interaction.h"



class PassCmdToMoveit{
  private:
    std::unique_ptr<robot_move_api::RosMove> robotCmd;
    std::unique_ptr<robot_move_api::RobotJointStatePub> robotJointStatePub;

    ros::Publisher currentPositionPub;
    ros::Subscriber jointStatesSub;
    ros::Subscriber jointTargetSub;
    ros::Subscriber cartesianSub;
    const int publishHz = 10;
    bool isLive =true;


  public:
    PassCmdToMoveit(ros::NodeHandle nodeHandle,std::vector<double> initialJointPosition){
      
      auto initialJointPositionRad = robot_move_api::RosMove::joint_degs_to_joint_rads(initialJointPosition);
      robotJointStatePub = std::make_unique<robot_move_api::RobotJointStatePub>(nodeHandle,initialJointPositionRad);

      robotCmd = std::make_unique<robot_move_api::RosMove>("tm_robot");

      jointStatesSub = nodeHandle.subscribe("tm_moveit_joint_states", 1000, &PassCmdToMoveit::set_joint_states_callback, this);
      jointTargetSub = nodeHandle.subscribe("tm_moveit_joint_target", 1000,&PassCmdToMoveit::set_joint_target_callback,this);
      cartesianSub = nodeHandle.subscribe("tm_moveit_cartiesan_target", 1000,&PassCmdToMoveit::set_cartesian_target_callback,this);
      currentPositionPub = nodeHandle.advertise<geometry_msgs::Pose>("tm_moveit_current_position",1000);

      std::thread(&PassCmdToMoveit::current_position_pub,this).detach();
    }

    ~PassCmdToMoveit(){
      isLive =false;
    }
    
    void set_joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg){
      std::cout<<"set_joint_states_callback is called"<<std::endl;
      auto jointPosition = robot_move_api::RosMove::joint_degs_to_joint_rads(msg->position);
  
      robotJointStatePub->set_new_joint_position(jointPosition);
    }
    void set_joint_target_callback(const sensor_msgs::JointState::ConstPtr& msg){
      std::cout<<"set_joint_target_callback is called"<<std::endl;
      auto jointTargetRad = robot_move_api::RosMove::joint_degs_to_joint_rads(msg->position);

      robotCmd->joint_move(jointTargetRad,false);
    }
    void set_cartesian_target_callback(const geometry_msgs::Pose::ConstPtr& msg){
      std::cout<<"set_cartesian_target_callback is called"<<std::endl;
      robotCmd->cartesian_move(*msg,false);
    }
    void current_position_pub(){
      
      ros::Rate loop_rate(publishHz);  
      while (ros::ok() && isLive){
        auto currentEndEffectorPosition = robotCmd->current_end_effector_position();
        currentPositionPub.publish(currentEndEffectorPosition);
        ros::spinOnce();
        loop_rate.sleep();
      }
      std::cout<<"end publish"<<std::endl;
    }
    
};

int main(int argc, char** argv){
  ros::init(argc, argv, "pass_moveit");
  ros::NodeHandle nodeHandle;

  std::vector<double> initialJointPosition{0.0,0.0,90.0,0.0,90.0,0.0};
  
  std::unique_ptr<PassCmdToMoveit> passCmdToMoveit = std::make_unique<PassCmdToMoveit>(nodeHandle,initialJointPosition);

  ros::spin();
  return 0;

}