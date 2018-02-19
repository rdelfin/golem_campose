cmake_minimum_required(VERSION 2.8.3)
project(campose_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
)


add_message_files(
  FILES
  FramePoses.msg
  Keypoint.msg
  Person.msg
  PersonAngles.msg
  PersonPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy message_runtime std_msgs
)
