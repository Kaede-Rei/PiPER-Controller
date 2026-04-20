cmake_minimum_required(VERSION 3.0.2)
project(piper_gui)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  geometry_msgs
  piper_msgs2
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/piper_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)