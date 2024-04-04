cmake_minimum_required(VERSION 3.0.2)
project(planning_scene_spawner)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  tf2_ros
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES $(PROJECT_NAME)
 CATKIN_DEPENDS moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools roscpp tf2_ros  geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/planning_scene_spawn.cpp
)
add_executable(planning_scene_node  nodes/planning_scene_node.cpp)
add_dependencies(planning_scene_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_scene_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
