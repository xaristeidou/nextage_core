/**
 * @file planning_scene_node.cpp
 * @author Gkrizis Christos (gkrizis@lms.mech.upatras.gr)
 * @brief A demo node that utilizes the planning scene spawner library
 * @version 0.1
 * @date 2024-04-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <csignal>
#include <planning_scene_spawner/planning_scene_spawn.h>
#include <ros/ros.h>

std::unique_ptr<PlanningSceneSpawn> planning_scene_ptr;

void signalHandler(int signum) {
  ROS_INFO("Interrupt signal (%d) received", signum);
  planning_scene_ptr.reset();
  exit(signum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_scene_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  planning_scene_ptr = std::make_unique<PlanningSceneSpawn>(nh, nh_local);
  if (!planning_scene_ptr->initialize()) {
    ROS_ERROR("Failed to initialize planning scene");
    return -1;
  }
  signal(SIGINT, signalHandler);
  return 0;
}