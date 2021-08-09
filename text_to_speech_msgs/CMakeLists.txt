cmake_minimum_required(VERSION 3.0.2)
project(text_to_speech_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
)

add_action_files(
   DIRECTORY
   action
   FILES
   Play.action
   Speak.action
)


## Generate services
add_service_files(
  DIRECTORY
  srv
  FILES
  GetStatus.srv
  Speak.srv
  Play.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
