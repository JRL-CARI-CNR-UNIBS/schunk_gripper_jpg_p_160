cmake_minimum_required(VERSION 3.8)
project(schunk_gripper_jpg_p_160_description)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# find_package(message_generation REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/MoveGripper.srv
  DEPENDENCIES std_msgs
)

install(
    DIRECTORY
        config 
        launch 
        meshes
        scripts
        srv
        urdf
    DESTINATION
        share/${PROJECT_NAME}
)

ament_package()
