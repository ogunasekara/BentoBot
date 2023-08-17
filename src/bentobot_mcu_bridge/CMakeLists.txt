cmake_minimum_required(VERSION 3.0.2)
project(bentobot_mcu_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MCUInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# install python script
catkin_install_python(PROGRAMS
  scripts/bentobot_mcu_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)