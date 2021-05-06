cmake_minimum_required(VERSION 3.0.2)
project(text_to_speech_msgs)

find_package(catkin REQUIRED COMPONENTS
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
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
