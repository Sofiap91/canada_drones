cmake_minimum_required(VERSION 3.0.2)
project(drone_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  drone_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vale_controller
  CATKIN_DEPENDS roscpp drone_msgs geometry_msgs
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

