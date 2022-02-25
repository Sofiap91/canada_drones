cmake_minimum_required(VERSION 3.0.2)
project(drone_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Poses.msg
)

 add_service_files(
   FILES
   Commands.srv
   PosesList.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)