cmake_minimum_required(VERSION 2.8.3)
project(jsk_mbzirc_msgs)
find_package(catkin REQUIRED std_msgs sensor_msgs geometry_msgs message_generation)

add_message_files(
  FILES 
  ProjectionMatrix.msg
  Rect.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

