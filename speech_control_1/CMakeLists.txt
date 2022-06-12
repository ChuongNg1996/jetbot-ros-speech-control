cmake_minimum_required(VERSION 3.0.2)
project(speech_control_1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED std_msgs)


catkin_package(
	CATKIN_DEPENDS  std_msgs 
)
catkin_install_python(PROGRAMS src/speech_command.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

