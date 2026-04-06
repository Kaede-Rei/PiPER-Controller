cmake_minimum_required(VERSION 3.0.2)
project(piper_gui)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  piper_msgs2
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/piper_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)