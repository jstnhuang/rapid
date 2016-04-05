#include "rapid_manipulation/pick_place.h"

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <map>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Pose.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/Grasp.h"
#include "moveit_msgs/PlanningScene.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "shape_msgs/SolidPrimitive.h"
#include "shape_tools/solid_primitive_dims.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "rapid_manipulation/arm.h"
#include "rapid_perception/scene.h"
#include "rapid_utils/math.h"
#include "rapid_viz/markers.h"

using boost::shared_ptr;
using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
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
  const vector<Object>* objects = table->objects();
  for (size_t i = 0; i < objects->size(); ++i) {
    const Object& object = (*objects)[i];
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
  const vector<Object>* objects = scene_.GetPrimarySurface()->objects();
  for (size_t i = 0; i < objects->size(); ++i) {
    const Object& obj = (*objects)[i];
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
  geometry_msgs::PoseStamped grasp_pose = pre_grasp;
  double grasp_distance = 0;
  ros::param::param<double>("grasp_distance", grasp_distance, 0.08);
  grasp_pose.pose.position.x += pregrasp_distance - grasp_distance;
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
  ROS_INFO("Attempting post-grasp");
  geometry_msgs::PoseStamped post_grasp = grasp_pose;
  double postgrasp_lift = 0;
  ros::param::param<double>("postgrasp_lift", postgrasp_lift, 0.15);
  post_grasp.pose.position.z += postgrasp_lift;
  post_grasp.pose.orientation.x = 0;
  post_grasp.pose.orientation.y = 0;
  post_grasp.pose.orientation.z = 0;
  post_grasp.pose.orientation.w = 1;
  success = arm_->MoveToPoseGoal(post_grasp);
  if (!success) {
    ROS_INFO("Post grasp failed");
    return PickError(PickError::POST_GRASP_FAILED);
  }
  ROS_INFO("Pick succeeded");

  return PickError(PickError::SUCCESS);
}

Placer::Placer(shared_ptr<ArmInterface> arm,
               shared_ptr<GripperInterface> gripper)
    : nh_(),
      marker_pub_(nh_.advertise<visualization_msgs::Marker>(
          "visualization_marker", 10)),
      arm_(arm),
      gripper_(gripper) {}

bool Placer::Place(Object& obj, Tabletop& table) {
  // Naive, proof of concept place.
  ROS_INFO("Attempting place");

  // Search for an empty spot on the table to place the object.
  // Could use a spatial data structure to find an empty spot in the future.
  // For now, we will just pick a spot at random and check that it's far away
  // from any objects.
  int max_tries = 0;
  ros::param::param<int>("place_max_tries", max_tries, 10);
  for (int num_tries = 0; num_tries < max_tries; ++num_tries) {
    ROS_INFO("Sampling placement location");
    geometry_msgs::PoseStamped location;
    geometry_msgs::PointStamped loc_position;
    SampleRandomPlacement(obj.scale(), table, &loc_position);
    location.header = loc_position.header;
    location.pose.position = loc_position.point;
    location.pose.orientation.w = 1;

    rapid::viz::Marker marker =
        rapid::viz::Marker::Box(marker_pub_, location, obj.scale());
    marker.SetNamespace("sampled_placement");
    marker.SetColor(1, 1, 0, 0.5);
    marker.Publish();

    bool success = false;

    // Pre-place
    ROS_INFO("Attempting pre-place");
    geometry_msgs::PoseStamped pre_place = location;
    double preplace_distance = 0;  // Height above placement location to start.
    ros::param::param<double>("preplace_distance", preplace_distance, 0.15);
    pre_place.pose.position.z += preplace_distance;
    double place_offset =
        0;  // Offset is distance from wrist link to object midpoint.
    ros::param::param<double>("place_offset", place_offset, 0.08);
    pre_place.pose.position.x -= place_offset;
    pre_place.pose.orientation.x = 0;
    pre_place.pose.orientation.y = 0;
    pre_place.pose.orientation.z = 0;
    pre_place.pose.orientation.w = 1;
    success = arm_->MoveToPoseGoal(pre_place);
    if (!success) {
      ROS_WARN("Failed to move to pre-place location.");
      continue;
    }

    // Place
    ROS_INFO("Attempting place");
    geometry_msgs::PoseStamped place = pre_place;
    place.pose.position.z -= preplace_distance;
    place.pose.orientation.x = 0;
    place.pose.orientation.y = 0;
    place.pose.orientation.z = 0;
    place.pose.orientation.w = 1;
    success = arm_->MoveToPoseGoal(place);
    if (!success) {
      ROS_WARN("Failed to move to place location.");
      continue;
    }
    gripper_->Open();

    // Post-place
    ROS_INFO("Attempting post-place");
    geometry_msgs::PoseStamped post_place = place;
    post_place.pose.position.z += preplace_distance;
    post_place.pose.orientation.x = 0;
    post_place.pose.orientation.y = 0;
    post_place.pose.orientation.z = 0;
    post_place.pose.orientation.w = 1;
    success = arm_->MoveToPoseGoal(post_place);
    if (!success) {
      ROS_WARN("Failed to move to post-place location.");
      continue;
    }
    return true;
  }

  ROS_ERROR("Failed to place object %s on table %s.", obj.name().c_str(),
            table.name().c_str());
  return false;
}

bool SampleRandomPlacement(const Vector3& object_scale,
                           const rapid::perception::Tabletop& table,
                           geometry_msgs::PointStamped* location) {
  int max_tries = 0;
  ros::param::param<int>("sample_placement_max_tries", max_tries, 100);
  geometry_msgs::PoseStamped table_ps = table.pose();
  double x_range = table.scale().x - object_scale.x;
  double start_x = -table.scale().x / 2 + object_scale.x / 2;
  double y_range = table.scale().y - object_scale.y;
  double start_y = -table.scale().y / 2 + object_scale.y / 2;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster table_broadcaster;
  tf::Transform table_frame;
  tf::Vector3 table_origin(table_ps.pose.position.x, table_ps.pose.position.y,
                           table_ps.pose.position.z);
  tf::Quaternion table_orientation(
      table_ps.pose.orientation.x, table_ps.pose.orientation.y,
      table_ps.pose.orientation.z, table_ps.pose.orientation.w);
  table_frame.setOrigin(table_origin);
  table_frame.setRotation(table_orientation);
  table_broadcaster.sendTransform(tf::StampedTransform(
      table_frame, ros::Time::now(), "base_footprint", "table"));
  ros::Duration(0.5).sleep();
  double z = table.scale().z / 2 + object_scale.z / 2;

  for (int i = 0; i < max_tries; ++i) {
    PointStamped pos;
    pos.header.frame_id = "table";
    std::srand(ros::Time::now().toNSec());
    pos.point.x =
        start_x + static_cast<double>(std::rand()) / RAND_MAX * x_range;
    pos.point.y =
        start_y + static_cast<double>(std::rand()) / RAND_MAX * y_range;
    pos.point.z = z;

    PointStamped transformed;
    tf_listener.transformPoint("base_footprint", pos, transformed);

    ROS_INFO("Try %d (base frame): x=%f, y=%f, z=%f", i, transformed.point.x,
             transformed.point.y, transformed.point.z);

    // Check that the object doesn't intersect with all other objects.
    bool intersect = false;
    const vector<Object>* objects = table.objects();
    Vector3 inflated_object = object_scale;  // Create an inflated object to
                                             // account for gripper width.
    inflated_object.x += 0.05;
    inflated_object.y += 0.05;
    for (size_t obj_i = 0; obj_i < objects->size(); ++obj_i) {
      intersect = rapid::utils::AabbXYIntersect(
          transformed.point, inflated_object,
          (*objects)[obj_i].pose().pose.position, (*objects)[obj_i].scale());
      if (intersect) {
        break;
      }
    }
    if (intersect) {
      continue;
    }

    *location = transformed;

    return true;
  }
  return false;
}
}  // namespace manipulation
}  // namespace rapid
