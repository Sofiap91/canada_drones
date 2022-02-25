#include "drone_controller.h"
#include "ros/ros.h"
#include <cstddef>

Drone::Drone(ros::NodeHandle *nodehandle) : nh_(*nodehandle) {

  initializeClients();

  droneController();
}

void Drone::initializeClients() {
  drone_srv_client_ =
      nh_.serviceClient<drone_msgs::Commands>("drone_control_srv");
}

void Drone::droneController() {
  //
  ROS_INFO("Initializing...");
  ros::Duration(0.1).sleep();

  ROS_INFO("Drone taking off...");
  current_command_.request.command = "takeoff";
  drone_srv_client_.call(current_command_);

  ros::Duration(2).sleep();
  ROS_INFO("Drone landing...");
  current_command_.request.command = "land";
  drone_srv_client_.call(current_command_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_controller");
  ros::NodeHandle nh;

  ros::Rate rate(10);
  ros::AsyncSpinner spinner(10);
  spinner.start();

  Drone droneObject(&nh);

  ros::waitForShutdown();
  return 0;
}