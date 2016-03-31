#include "rapid_manipulation/pick_place.h"

#include <iostream>
#include <map>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Pose.h"
#include "shape_msgs/SolidPrimitive.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/Grasp.h"
#include "moveit_msgs/PlanningScene.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "shape_tools/solid_primitive_dims.h"

#include "rapid_manipulation/arm.h"
#include "rapid_perception/scene.h"

using boost::shared_ptr;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using moveit_msgs::CollisionObject;
using moveit_msgs::PlanningScene;
using moveit_msgs::Grasp;
using rapid::perception::Object;
using rapid::perception::Scene;
using rapid::perception::Tabletop;
using shape_msgs::SolidPrimitive;
using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

namespace rapid {
namespace manipulation {
const std::string PickError::SUCCESS = "success";
const std::string PickError::OBJ_NOT_FOUND = "obj_not_found";
const std::string PickError::PRE_GRASP_FAILED = "pre_grasp_failed";
const std::string PickError::GRASP_FAILED = "grasp_failed";
const std::string PickError::POST_GRASP_FAILED = "post_grasp_failed";

Picker::Picker(shared_ptr<ArmInterface> arm,
               shared_ptr<GripperInterface> gripper)
    : nh_(),
      co_pub_(nh_.advertise<CollisionObject>("collision_object", 10)),
      ps_pub_(nh_.advertise<PlanningScene>("planning_scene", 10)),
      tf_listener_(),
      scene_(),
      arm_(arm),
      gripper_(gripper) {
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
  co_pub_.publish(remove);

  PlanningScene ps;
  ps.world.collision_objects.push_back(remove);
  ps.is_diff = true;
  // ps_pub_.publish(ps);

  // Add updated object.
  CollisionObject updated = obj;
  updated.operation = CollisionObject::ADD;
  ps.world.collision_objects.clear();
  ps.world.collision_objects.push_back(updated);
  ps.is_diff = true;
  // ps_pub_.publish(ps);
  co_pub_.publish(obj);
}

void Picker::UpdatePlanningScene(Scene& scene) {
  scene_ = scene;

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
    cout << "Adding object " << co.id << " to planning scene." << endl;
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

  bool success = false;
  PoseStamped obj_pose_original;
  vector<Object> objects = scene_.GetPrimarySurface()->objects();
  for (size_t i = 0; i < objects.size(); ++i) {
    const Object& obj = objects[i];
    if (obj.name() == obj_name) {
      obj_pose_original = obj.pose();
      success = true;
      break;
    }
  }
  if (!success) {
    return PickError(PickError::OBJ_NOT_FOUND);
  }

  PoseStamped obj_pose;
  tf_listener_.transformPose("base_footprint", obj_pose_original, obj_pose);

  // Pre-grasp
  ROS_INFO("Attempting pre-grasp");
  geometry_msgs::PoseStamped pre_grasp = obj_pose;
  double pregrasp_distance = 0;
  ros::param::param<double>("pregrasp_distance", pregrasp_distance, 0.3);
  pre_grasp.pose.position.x -= pregrasp_distance;
  pre_grasp.pose.orientation.x = 0;
  pre_grasp.pose.orientation.y = 0;
  pre_grasp.pose.orientation.z = 0;
  pre_grasp.pose.orientation.w = 1;
  success = arm_->MoveToPoseGoal(pre_grasp);
  if (!success) {
    return PickError(PickError::PRE_GRASP_FAILED);
  }
  gripper_->Open();

  // Grasp
  ROS_INFO("Attempting grasp");
  geometry_msgs::PoseStamped grasp_pose = obj_pose;
  double grasp_distance = 0;
  ros::param::param<double>("grasp_distance", grasp_distance, 0.08);
  grasp_pose.pose.position.x -= grasp_distance;
  grasp_pose.pose.orientation.x = 0;
  grasp_pose.pose.orientation.y = 0;
  grasp_pose.pose.orientation.z = 0;
  grasp_pose.pose.orientation.w = 1;
  success = arm_->MoveToPoseGoal(grasp_pose);
  if (!success) {
    return PickError(PickError::GRASP_FAILED);
  }
  gripper_->Close();

  // Post-grasp
  ROS_INFO("Attempting pose-grasp");
  geometry_msgs::PoseStamped post_grasp = obj_pose;
  double postgrasp_lift = 0;
  ros::param::param<double>("postgrasp_lift", postgrasp_lift, 0.15);
  post_grasp.pose.position.z += postgrasp_lift;
  post_grasp.pose.orientation.x = 0;
  post_grasp.pose.orientation.y = 0;
  post_grasp.pose.orientation.z = 0;
  post_grasp.pose.orientation.w = 1;
  success = arm_->MoveToPoseGoal(post_grasp);
  if (!success) {
    return PickError(PickError::POST_GRASP_FAILED);
  }

  // moveit::planning_interface::PlanningSceneInterface psi;
  // vector<string> obj_query;
  // obj_query.push_back(obj_name);
  // map<string, Pose> obj_poses = psi.getObjectPoses(obj_query);
  // if (obj_poses.find(obj_name) == obj_poses.end()) {
  //  return PickError(PickError::OBJ_NOT_FOUND);
  //}
  // const Pose& obj_pose = obj_poses[obj_name];

  // geometry_msgs::PoseStamped p;
  // p.header.frame_id = "base_footprint";
  // p.pose.position.x = obj_pose.pose.position.x;
  // p.pose.position.y = obj_pose.pose.position.y;
  // p.pose.position.z = obj_pose.pose.position.z;
  // p.pose.orientation.x = 0;
  // p.pose.orientation.y = 0;
  // p.pose.orientation.z = 0;
  // p.pose.orientation.w = 1;
  // moveit_msgs::Grasp g;
  // g.grasp_pose = p;

  // g.pre_grasp_approach.direction.vector.x = 1.0;
  // g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  // g.pre_grasp_approach.min_distance = 0.1;
  // g.pre_grasp_approach.desired_distance = 0.3;

  // g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  // g.post_grasp_retreat.direction.vector.z = 1.0;
  // g.post_grasp_retreat.min_distance = 0.1;
  // g.post_grasp_retreat.desired_distance = 0.25;

  // g.pre_grasp_posture.joint_names.resize(1, "r_gripper_joint");
  // g.pre_grasp_posture.points.resize(1);
  // g.pre_grasp_posture.points[0].positions.resize(1);
  // g.pre_grasp_posture.points[0].positions[0] = 0.08;

  // g.grasp_posture.joint_names.resize(1, "r_gripper_joint");
  // g.grasp_posture.points.resize(1);
  // g.grasp_posture.points[0].positions.resize(1);
  // g.grasp_posture.points[0].positions[0] = 0;

  // grasps.push_back(g);
  // group_->setSupportSurfaceName(support_name);
  // group_->pick(obj_name, grasps);
  return PickError(PickError::SUCCESS);
}
}  // namespace manipulation
}  // namespace rapid
