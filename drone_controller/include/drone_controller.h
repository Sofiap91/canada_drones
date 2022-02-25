#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include "ros/ros.h"

#include <drone_msgs/Commands.h>

class Drone {
public:
  Drone(ros::NodeHandle *nodehandle);

  void droneController();

private:
  ros::NodeHandle nh_;

  ros::ServiceClient drone_srv_client_;

  drone_msgs::Commands current_command_;

  void initializeClients();
};

#endif