/**
 * @file planning_scene_spawn.cpp
 * @author Gkrizis Christos (gkrizis@lms.mech.upatras.gr)
 * @brief Generic planning scene spawner library
 * @version 0.1
 * @date 2024-04-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <planning_scene_spawner/planning_scene_spawn.h>

PlanningSceneSpawn::PlanningSceneSpawn(ros::NodeHandle &nh,
                                       ros::NodeHandle &nh_local)
    : nh_(nh), nh_local_(nh_local) {}

PlanningSceneSpawn::~PlanningSceneSpawn() {}

bool PlanningSceneSpawn::initialize(void) {
  robot_model_ = robot_model_loader_.getModel();
  if (robot_model_ == nullptr) {
    ROS_ERROR("Failed to load robot model");
    return false;
  }

  planning_scene_ptr_ =
      std::make_shared<planning_scene::PlanningScene>(robot_model_);

  collision_objects_.resize(0);
  std_msgs::ColorRGBA color;
  shape_msgs::Mesh shape_mesh;
  shapes::ShapeMsg shape_msg;
  shapes::Mesh *shape_mesh_ptr;
  moveit_msgs::CollisionObject collision_object;
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  planning_scene_ptr_->removeAllCollisionObjects();
  ROS_INFO("Initialized planning scene");

  // Planning scene mesh pose
  geometry_msgs::Pose pose;
  pose.position.x = 0.20;
  pose.position.y = 0.30;
  pose.position.z = -0.85;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = -0.707;
  pose.orientation.w = 0.707;

  shape_mesh_ptr = shapes::createMeshFromResource(
      "package://planning_scene_spawner/resources/elux.stl");
  shapes::constructMsgFromShape(shape_mesh_ptr, shape_msg);
  shape_mesh = boost::get<shape_msgs::Mesh>(shape_msg);
  collision_object.header.frame_id = "base_link";
  collision_object.id = "tables_structure";
  collision_object.meshes[0] = shape_mesh;
  collision_object.mesh_poses[0] = pose;
  collision_object.operation = collision_object.ADD;
  collision_objects_.push_back(collision_object);

  planning_scene_.name = "demo_planning_scene";
  planning_scene_.robot_state.is_diff = false;
  planning_scene_.world.collision_objects = collision_objects_;
  planning_scene_.is_diff = true;

  ROS_INFO("Added %d objects to planning scene", collision_objects_.size());
  planning_scene_ptr_->setPlanningSceneDiffMsg(planning_scene_);
  planning_scene_publisher_.publish(planning_scene_);
  return true;
}
