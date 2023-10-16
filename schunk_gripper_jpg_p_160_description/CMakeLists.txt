cmake_minimum_required(VERSION 3.0.2)
project(schunk_gripper_jpg_p_160_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

add_service_files(
  FILES
  MoveGripper.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
