cmake_minimum_required(VERSION 2.8.3)
project(pathplanning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  pathplaning_msgs
)

## Generate messages in the 'msg' folder

#add_message_files( FILES  track_path.msg)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   #   diagnostic_msgs#   gps_msgs#   jsk_recognition_msgs#   little_ant_msgs#   
   std_msgs
   nav_msgs
   geometry_msgs
 )


catkin_package(
  CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES driverless
#  CATKIN_DEPENDS ant_math diagnostic_msgs gps_msgs jsk_recognition_msgs little_ant_msgs message_filters message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(generate_vehicle_pose src/generate_vehicle_pose.cpp)
target_link_libraries(generate_vehicle_pose ${catkin_LIBRARIES})
add_dependencies(generate_vehicle_pose ${PROJECT_NAME}_generate_messages_cpp)


add_executable(pathplan src/pathplan.cpp src/planfunction.cpp)
target_link_libraries(pathplan ${catkin_LIBRARIES})
add_dependencies(pathplan ${PROJECT_NAME}_generate_messages_cpp)





