#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include "ros/ros.h"

#include <drone_msgs/Commands.h>
#include <drone_msgs/Poses.h>
#include <drone_msgs/PosesList.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class Drone {
public:
  // Constructor of the class.
  Drone(ros::NodeHandle *nodehandle);

  void droneController();

private:
  ros::NodeHandle nh_;

  /**
   * The robot states enum. Contains all the possible states
   **/
  enum controller_states {
    land,
    takeoff,
    move,
    get_poses,
    idle
  } current_state_;

  /**
   * Global variables
   **/
  controller_states last_state_;

  ros::ServiceClient drone_srv_client_;
  ros::ServiceClient pos_srv_client_;
  ros::Publisher pos_pub_;
  ros::Subscriber pos_sub_;

  drone_msgs::Commands current_command_;
  drone_msgs::PosesList pose_list_;
  drone_msgs::Poses actual_pose_;
  geometry_msgs::Twist current_pose_;
  geometry_msgs::Pose drone_curr_pose_;
  geometry_msgs::Pose last_pose_;
  int pose_size_;
  int iterator_ = 0;

  /**
   * Initialization of all ROS Clients.
   */
  void initialize();

  /**
   * Function to land the drone
   */
  void landDrone();

  /**
   * Function to start the drone
   */
  void takeoffDrone();

  /**
   * This function calls the STL Parser service
   */
  void getPosList();

  /**
   * This function moves the drone
   * @param pose is the position to which the drone should move next
   */
  void moveDrone(geometry_msgs::Twist pose);

  /**
   * This will return the drone position at all times
   * @param pos contain the values published by the drone position topic.
   */
  void getDronePosition(const geometry_msgs::PoseConstPtr &pos);

  /**
   * FSM state helper function.
   */
  void changeStateTo(controller_states new_state);
};

#endif