cmake_minimum_required(VERSION 3.0.2)
project(ccavt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  ShareInfo.msg
  Path.msg
  Obstacle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 LIBRARIES ccavt
 CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  v2x
  ${catkin_INCLUDE_DIRS}
)


set_source_files_properties(PROPERTIES LANGUAGE CXX)
