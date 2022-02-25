cmake_minimum_required(VERSION 3.0.2)
project(drone_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  drone_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vale_controller
  CATKIN_DEPENDS roscpp drone_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

