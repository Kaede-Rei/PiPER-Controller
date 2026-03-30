cmake_minimum_required(VERSION 3.0.2)
project(piper_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  piper_msgs_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/piper_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)