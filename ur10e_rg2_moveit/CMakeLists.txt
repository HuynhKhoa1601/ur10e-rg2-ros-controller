cmake_minimum_required(VERSION 3.1.3)
project(ur10e_rg2_moveit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
  moveit_msgs
  sensor_msgs
  controller_manager
  roslaunch
  trajectory_msgs
)

roslaunch_add_file_check(launch)

# Add custom messages if you have them
add_message_files(DIRECTORY msg)

# Add your service
add_service_files(
  FILES
  PoseEstimationService.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  scripts/mover.py
  scripts/trajectory_subscriber.py
  scripts/sim_real_pnp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

