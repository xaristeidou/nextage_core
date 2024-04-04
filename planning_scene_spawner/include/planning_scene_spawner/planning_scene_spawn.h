/**
 * @file planning_scene_spawn.h
 * @author Gkrizis Christos (gkrizis@lms.mech.upatras.gr)
 * @brief Generic planning scene spawner library
 * @version 0.1
 * @date 2024-04-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <boost/scoped_ptr.hpp>
#include <limits>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <utility>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/SolidPrimitive.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class PlanningSceneSpawn {
public:
  /**
   * @brief Construct a new Planning Scene Spawn object
   *
   * @param nh
   * @param nh_local
   */
  PlanningSceneSpawn(ros::NodeHandle &nh, ros::NodeHandle &nh_local);
  /**
   * @brief Destroy the Planning Scene Spawn object
   *
   */
  ~PlanningSceneSpawn();
  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool initialize(void);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  const std::string robot_description_ = "robot_description";

  moveit_msgs::PlanningScene planning_scene_;
  planning_scene::PlanningScenePtr planning_scene_ptr_;
  planning_interface::PlannerManagerPtr planner_instance_;

  std::vector<moveit_msgs::CollisionObject> collision_objects_;

  ros::Publisher planning_scene_publisher_ =
      nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1, true);

  robot_model::RobotModelPtr robot_model_;
  robot_model_loader::RobotModelLoader robot_model_loader_;

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>>
      planner_plugin_loader_;

  std::string planner_plugin_name_;
};