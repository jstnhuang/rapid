#include "rapid_manipulation/pick_place.h"

#include <iostream>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "shape_msgs/SolidPrimitive.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/Grasp.h"
#include "moveit_msgs/PlanningScene.h"
#include "shape_tools/solid_primitive_dims.h"

#include "rapid_perception/scene.h"

using boost::shared_ptr;
using moveit_msgs::CollisionObject;
using moveit_msgs::PlanningScene;
using moveit_msgs::Grasp;
using rapid::perception::Object;
using rapid::perception::Scene;
using rapid::perception::Tabletop;
using shape_msgs::SolidPrimitive;
using std::string;
using std::vector;

namespace rapid {
namespace manipulation {
const std::string PickError::SUCCESS = "success";

Picker::Picker(boost::shared_ptr<moveit::planning_interface::MoveGroup> group)
    : group_(group),
      nh_(),
      co_pub_(nh_.advertise<CollisionObject>("collision_object", 10)),
      ps_pub_(nh_.advertise<PlanningScene>("planning_scene", 10)) {
  while (ps_pub_.getNumSubscribers() < 1) {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
    ROS_INFO("Waiting for subscribers");
  }
  ROS_INFO("Planning scene publisher ready");
}

void Picker::UpdatePlanningSceneTopic(const string& id,
                                      const CollisionObject& obj) {
  // Remove old object.
  CollisionObject remove;
  remove.id = id;
  remove.operation = CollisionObject::REMOVE;

  PlanningScene ps;
  ps.world.collision_objects.push_back(remove);
  ps.is_diff = true;
  ps_pub_.publish(ps);

  // Add updated object.
  CollisionObject updated = obj;
  updated.operation = CollisionObject::ADD;
  ps.world.collision_objects.clear();
  ps.world.collision_objects.push_back(updated);
  ps.is_diff = true;
  ps_pub_.publish(ps);
}

void Picker::UpdatePlanningScene(Scene& scene) {
  PlanningScene ps;
  CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";

  // Update table.
  shared_ptr<Tabletop> table = scene.GetPrimarySurface();
  co.operation = CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(
      shape_tools::SolidPrimitiveDimCount<SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[SolidPrimitive::BOX_X] = table->scale().x;
  co.primitives[0].dimensions[SolidPrimitive::BOX_Y] = table->scale().y;
  co.primitives[0].dimensions[SolidPrimitive::BOX_Z] = table->scale().z;
  co.primitive_poses.resize(1);
  co.primitive_poses[0] = table->pose().pose;
  UpdatePlanningSceneTopic("table", co);

  // Update objects
  vector<Object> objects = table->objects();
  for (size_t i = 0; i < objects.size(); ++i) {
    const Object& object = objects[i];
    co.id = object.name();
    co.operation = CollisionObject::ADD;
    co.primitives[0].type = SolidPrimitive::BOX;
    co.primitives[0].dimensions[SolidPrimitive::BOX_X] = object.scale().x;
    co.primitives[0].dimensions[SolidPrimitive::BOX_Y] = object.scale().y;
    co.primitives[0].dimensions[SolidPrimitive::BOX_Z] = object.scale().z;
    co.header = object.pose().header;
    co.header.stamp = ros::Time::now();
    co.primitive_poses[0] = object.pose().pose;
    UpdatePlanningSceneTopic(object.name(), co);
  }
}

PickError Picker::Pick(const string& obj_name, const string& support_name) {
  // Do a naive, proof-of-concept pick.
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.32;
  p.pose.position.y = -0.7;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  g.pre_grasp_approach.min_distance = 0.2;
  g.pre_grasp_approach.desired_distance = 0.4;

  g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

  g.pre_grasp_posture.joint_names.resize(1, "r_gripper_joint");
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 1;

  g.grasp_posture.joint_names.resize(1, "r_gripper_joint");
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0;

  grasps.push_back(g);
  group_->setSupportSurfaceName("table");
  group_->pick("part", grasps);
  return PickError::SUCCESS;
}
}  // namespace manipulation
}  // namespace rapid
