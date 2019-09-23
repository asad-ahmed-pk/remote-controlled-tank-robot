# Joystick teleop controller for using PS3 and Switch Pro Controller Joysticks

cmake_minimum_required(VERSION 2.8.3)
project(joystick_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp joystick_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(FILES JoystickInputMsg.msg)
#generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
CATKIN_DEPENDS roscpp joystick_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Exectubles
add_executable(joystick_controller src/joystick_controller_node.cpp
        src/JoystickInputHandler.cpp
        src/JoystickInputPublisher.cpp
)
add_dependencies(joystick_controller ${catkin_EXPORTED_TARGETS})

add_executable(js_test src/js_test.cpp src/JoystickInputHandler.cpp)

## Linking executables
target_link_libraries(joystick_controller ${catkin_LIBRARIES})
