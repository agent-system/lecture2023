cmake_minimum_required(VERSION 3.0.2)
project(synchronization)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
  std_msgs
  tf 
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/object_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/position_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

