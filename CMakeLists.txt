cmake_minimum_required(VERSION 2.8.3)
project(text_to_speech)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetStatus.srv
  Speak.srv
  Play.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES text_to_speech
  CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
