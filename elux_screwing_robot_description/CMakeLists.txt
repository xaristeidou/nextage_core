cmake_minimum_required(VERSION 3.0.2)
project(elux_screwing_robot_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  urdf
  xacro
  ur_description
  realsense2_description
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elux_screwing_robot_description
#  CATKIN_DEPENDS rviz urdf xacro roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)