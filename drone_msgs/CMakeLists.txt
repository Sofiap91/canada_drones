cmake_minimum_required(VERSION 3.0.2)
project(drone_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

 add_service_files(
   FILES
   Commands.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vale_msgs
  CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)