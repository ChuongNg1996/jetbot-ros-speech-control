cmake_minimum_required(VERSION 3.0.2)
project(speech_control_1)

find_package(catkin REQUIRED std_msgs)


catkin_package(
	CATKIN_DEPENDS  std_msgs 
)
catkin_install_python(PROGRAMS src/speech_command.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
${catkin_INCLUDE_DIRS}
)

