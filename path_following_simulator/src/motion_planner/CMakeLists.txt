cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ackermann_msgs
  message_generation
)

add_message_files(
   FILES
   State.msg
   LocalPath.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS scripts
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    ackermann_msgs
    message_runtime
  DEPENDS
)



