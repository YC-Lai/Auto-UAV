cmake_minimum_required(VERSION 3.0.2)
project(auto_uav_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  std_msgs
  sensor_msgs
)

include_directories(include)

#########################
##       MESSAGE       ##
#########################

##########################
##       SERVICE        ##
##########################
add_service_files(
  FILES
  Takeoff.srv
  Lock.srv
  Land.srv
  Connect.srv
)

generate_messages(DEPENDENCIES mavros_msgs geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime sensor_msgs std_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h")