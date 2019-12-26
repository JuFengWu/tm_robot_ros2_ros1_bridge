// Copyright 2019 Leo_Wu

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include "boost/date_time/posix_time/posix_time.hpp"

class TmRobotInteractiveMarker {
 private:
  geometry_msgs::Pose currentEEPosition;

  ros::NodeHandle nodeHandle;

  ros::Publisher targetPostionTalker;
  ros::Subscriber eePositionListener;

  bool isGetEEPostion;
  std::string markerName;

  void frame_callback(const ros::TimerEvent&);
  void process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  visualization_msgs::InteractiveMarkerControl& make_box_control(visualization_msgs::InteractiveMarker &msg);

  visualization_msgs::InteractiveMarkerControl set_interactive_marker_orientation_control
  (visualization_msgs::InteractiveMarkerControl control, double w, double x, double y, double z, std::string name);
  void ee_position(const geometry_msgs::Pose &eePostion);
  visualization_msgs::InteractiveMarkerControl set_interactive_marker_move_control(visualization_msgs::InteractiveMarkerControl control, std::string name);

 public:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  void make_6_dof_marker(tf::Vector3 initailPosition, tf::Quaternion initailRotation);
  geometry_msgs::Pose get_ee_postion();
  TmRobotInteractiveMarker();
  ~TmRobotInteractiveMarker();
};

TmRobotInteractiveMarker::TmRobotInteractiveMarker():isGetEEPostion(false), markerName("tm_control_marker") {
  targetPostionTalker = nodeHandle.advertise<geometry_msgs::Pose>("tm_interactive_reaction", 1000);
  eePositionListener = nodeHandle.subscribe("tm_current_ee_position", 1000, &TmRobotInteractiveMarker::ee_position, this);
  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls", "", false) );
  ros::Timer frame_timer = nodeHandle.createTimer(ros::Duration(0.01), &TmRobotInteractiveMarker::frame_callback, this);
}

void TmRobotInteractiveMarker::ee_position(const geometry_msgs::Pose &eePostion) {
  std_msgs::Header header;
  currentEEPosition = eePostion;
  isGetEEPostion = true;
}

TmRobotInteractiveMarker::~TmRobotInteractiveMarker() {
  server.reset();
}
void TmRobotInteractiveMarker::frame_callback(const ros::TimerEvent&) {
  static uint32_t counter = 0;
  static tf::TransformBroadcaster br;
  tf::Transform t;
  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(static_cast<float>(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, static_cast<float>(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

  counter++;
}
void TmRobotInteractiveMarker::process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {
  std::ostringstream s;
  std::ostringstream mouse_point_ss;
  geometry_msgs::Pose target;
  std_msgs::Header header;

  switch ( feedback->event_type ) {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

      target.position.x = feedback->pose.position.x;
      target.position.y = feedback->pose.position.y;
      target.position.z = feedback->pose.position.z;

      target.orientation.w = feedback->pose.orientation.w;
      target.orientation.x = feedback->pose.orientation.x;
      target.orientation.y = feedback->pose.orientation.y;
      target.orientation.z = feedback->pose.orientation.z;

      targetPostionTalker.publish(target);
      break;
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
      server->setPose(markerName, currentEEPosition, header);
      break;
  }
}
visualization_msgs::InteractiveMarkerControl& TmRobotInteractiveMarker::make_box_control(visualization_msgs::InteractiveMarker &msg ) {
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  msg.controls.push_back(control);

  return msg.controls.back();
}

visualization_msgs::InteractiveMarkerControl TmRobotInteractiveMarker::set_interactive_marker_orientation_control
  (visualization_msgs::InteractiveMarkerControl control, double w, double x, double y, double z, std::string name) {
  control.orientation.w = w;
  control.orientation.x = x;
  control.orientation.y = y;
  control.orientation.z = z;
  control.name = name;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

  return control;
}

visualization_msgs::InteractiveMarkerControl TmRobotInteractiveMarker::set_interactive_marker_move_control
(visualization_msgs::InteractiveMarkerControl control, std::string name) {
  control.name = name;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  return control;
}
void TmRobotInteractiveMarker::make_6_dof_marker(tf::Vector3 initailPosition, tf::Quaternion initailRotation) {
  const double SQRT2INV = 0.707106781;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  tf::pointTFToMsg(initailPosition, int_marker.pose.position);
  tf::quaternionTFToMsg(initailRotation, int_marker.pose.orientation);

  int_marker.scale = 0.2;

  int_marker.name = markerName;

  // insert a box
  make_box_control(int_marker);
  visualization_msgs::InteractiveMarkerControl control;
  int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  control = set_interactive_marker_orientation_control(control, SQRT2INV*1, SQRT2INV*1, 0, 0, "rotate_x");
  int_marker.controls.push_back(control);
  control = set_interactive_marker_move_control(control, "move_x");
  int_marker.controls.push_back(control);

  control = set_interactive_marker_orientation_control(control, SQRT2INV*1, 0, SQRT2INV*1, 0, "rotate_z");
  int_marker.controls.push_back(control);
  control = set_interactive_marker_move_control(control, "move_z");
  int_marker.controls.push_back(control);

  control = set_interactive_marker_orientation_control(control, SQRT2INV*1, 0, 0, SQRT2INV*1, "rotate_y");
  int_marker.controls.push_back(control);
  control = set_interactive_marker_move_control(control, "move_y");
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  auto callback =  boost::bind(&TmRobotInteractiveMarker::process_feedback, this, _1);
  server->setCallback(int_marker.name, callback);
}

geometry_msgs::Pose TmRobotInteractiveMarker::get_ee_postion() {
  ros::Rate loopRate(1);

  while (!isGetEEPostion) {
    std::cout<< "wait ee position" << std::endl;
    loopRate.sleep();
    ros::spinOnce();
  }
  return currentEEPosition;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_control_interactive_marker");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  std::unique_ptr<TmRobotInteractiveMarker> tmRobotInteractiveMarker = std::make_unique<TmRobotInteractiveMarker>();

  auto eePosition = tmRobotInteractiveMarker->get_ee_postion();

  tf::Vector3 initailPosition = tf::Vector3(eePosition.position.x, eePosition.position.y, eePosition.position.z);

  tf::Quaternion initailRotation = tf::Quaternion
      (eePosition.orientation.x, eePosition.orientation.y, eePosition.orientation.z, eePosition.orientation.w);

  tmRobotInteractiveMarker->make_6_dof_marker(initailPosition, initailRotation);

  tmRobotInteractiveMarker->server->applyChanges();

  ros::waitForShutdown();
}
