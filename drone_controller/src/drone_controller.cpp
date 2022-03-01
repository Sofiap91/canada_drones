#include "drone_controller.h"
#include "ros/ros.h"
#include <cstddef>

Drone::Drone(ros::NodeHandle *nodehandle) : nh_(*nodehandle) {
  initialize();
  current_state_ = takeoff;
  ros::Duration(1).sleep();
  droneController();
}

void Drone::initialize() {
  drone_srv_client_ =
      nh_.serviceClient<drone_msgs::Commands>("drone_control_srv");
  pos_srv_client_ =
      nh_.serviceClient<drone_msgs::PosesList>("stl_positions_srv");
  pos_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pos_sub_ = nh_.subscribe("drone/gt_pose", 1, &Drone::getDronePosition, this);
}

void Drone::droneController() {
  while (ros::ok()) {
    switch (current_state_) {
    case takeoff: {
      ROS_INFO("takeoff");
      takeoffDrone();
      changeStateTo(get_poses);
    } break;

    case land: {
      ROS_INFO("land");
      landDrone();
      changeStateTo(idle);
    } break;

    case move: {
      ROS_INFO("move");
      current_pose_.linear.x =
          pose_list_.response.pose_list[iterator_].poses[0];
      current_pose_.linear.y =
          pose_list_.response.pose_list[iterator_].poses[1];
      current_pose_.linear.z =
          pose_list_.response.pose_list[iterator_].poses[2];

      moveDrone(current_pose_);

      iterator_ += 1;
      ROS_INFO_STREAM(iterator_);

      if (iterator_ + 1 == pose_size_) {
        ROS_INFO("All positions reached");
        changeStateTo(land);
      }
    } break;

    case get_poses: {
      ROS_INFO("get_poses");
      getPosList();
      changeStateTo(move);
    } break;

    case idle: {
      ROS_INFO("IDLE");
      ros::Duration(1).sleep();
    } break;
    }
  }
}

void Drone::moveDrone(geometry_msgs::Twist target) {

  if (target.linear.x != drone_curr_pose_.position.x) {
    current_pose_.linear.x = target.linear.x - drone_curr_pose_.position.x;
  } else {
    current_pose_.linear.x = 0;
  }

  if (target.linear.y != drone_curr_pose_.position.y) {
    current_pose_.linear.y = target.linear.y - drone_curr_pose_.position.y;
  } else {
    current_pose_.linear.y = 0;
  }

  if (target.linear.z != drone_curr_pose_.position.z) {
    current_pose_.linear.z = target.linear.z - drone_curr_pose_.position.z;
  } else {
    current_pose_.linear.z = 0;
  }

  this->pos_pub_.publish(current_pose_);

  ros::Duration(1).sleep();
  current_pose_.linear.x = current_pose_.linear.y = current_pose_.linear.z = 0;
  this->pos_pub_.publish(current_pose_);
  ros::Duration(4).sleep();
}

void Drone::takeoffDrone() {
  ROS_INFO("Drone taking off...");
  current_command_.request.command = "takeoff";
  drone_srv_client_.call(current_command_);
  ros::Duration(2).sleep();
  last_pose_ = drone_curr_pose_;
}

void Drone::landDrone() {
  ROS_INFO("Drone landing...");
  current_pose_.linear.z = -1;
  this->pos_pub_.publish(current_pose_);
  while (1) {
    if (drone_curr_pose_.position.z <= 0.2)
      break;
  }
  current_pose_.linear.z = 0;
  this->pos_pub_.publish(current_pose_);

  current_command_.request.command = "land";
  drone_srv_client_.call(current_command_);
  ros::Duration(2).sleep();
}

void Drone::getPosList() {
  pos_srv_client_.call(pose_list_);
  pose_size_ = pose_list_.response.pose_list.size();
  // ROS_INFO_STREAM(pose_list_.response);
  ros::Duration(0.1).sleep();
  iterator_ = 0;
}

void Drone::getDronePosition(const geometry_msgs::PoseConstPtr &pos) {
  drone_curr_pose_.position = pos->position;
}

void Drone::changeStateTo(controller_states new_state) {
  last_state_ = current_state_;
  current_state_ = new_state;
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