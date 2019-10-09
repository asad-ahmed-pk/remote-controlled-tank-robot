# Joystick Messages

cmake_minimum_required(VERSION 2.8.3)
project(joystick_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(FILES JoystickInputMsg.msg)
generate_messages(DEPENDENCIES std_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
