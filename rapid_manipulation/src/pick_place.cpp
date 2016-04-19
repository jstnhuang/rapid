#include "rapid_manipulation/pick_place.h"

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <map>
#include <vector>

#include "agile_grasp/Grasp.h"
#include "agile_grasp/Grasps.h"
#include "agile_grasp/FindGrasps.h"
#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Quaternion.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/Grasp.h"
#include "moveit_msgs/PlanningScene.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "shape_msgs/SolidPrimitive.h"
#include "shape_tools/solid_primitive_dims.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include <Eigen/Dense>

#include "rapid_manipulation/arm.h"
#include "rapid_perception/scene.h"
#include "rapid_utils/math.h"
#include "rapid_viz/markers.h"

using agile_grasp::Grasp;
using agile_grasp::Grasps;
using boost::shared_ptr;
using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
using moveit_msgs::CollisionObject;
using moveit_msgs::PlanningScene;
using rapid::perception::Object;
using rapid::perception::Scene;
using rapid::perception::ScenePrimitive;
using rapid::perception::Tabletop;
using shape_msgs::SolidPrimitive;
using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

namespace rapid {
namespace manipulation {
Picker::Picker(ArmInterface* arm, GripperInterface* gripper)
    : nh_(),
      co_pub_(nh_.advertise<CollisionObject>("collision_object", 10)),
      ps_pub_(nh_.advertise<PlanningScene>("planning_scene", 10)),
      marker_pub_(nh_.advertise<visualization_msgs::Marker>(
          "visualization_marker", 10)),
      grasp_client_(nh_.serviceClient<agile_grasp::FindGrasps>(
          "find_grasps/find_grasps")),
      tf_listener_(),
      scene_(),
      arm_(arm),
      gripper_(gripper) {
  while (ps_pub_.getNumSubscribers() < 1) {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }
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

bool Picker::Pick(const Object& obj, double max_effort) {
  // Use agile_grasp to find some grasp poses.
  agile_grasp::FindGrasps grasps_srv;
  pcl::PointCloud<pcl::PointXYZ> colorless;
  pcl::copyPointCloud(*obj.GetCloud(), colorless);
  pcl::toROSMsg(colorless, grasps_srv.request.object);
  bool success = grasp_client_.call(grasps_srv);
  if (!success) {
    ROS_ERROR("Failed to find viable grasps.");
    return false;
  }

  // TODO(jstn): better grasp selection
  std::random_shuffle(grasps_srv.response.grasps.grasps.begin(),
                      grasps_srv.response.grasps.grasps.end());
  int num_grasps = 0;
  ros::param::param<int>("num_grasps", num_grasps, 10);

  bool grasped_object = false;
  for (size_t grasp_i = 0;
       grasp_i < std::min(static_cast<size_t>(num_grasps),
                          grasps_srv.response.grasps.grasps.size());
       ++grasp_i) {
    const Grasp& grasp = grasps_srv.response.grasps.grasps[grasp_i];

    std::string frame = grasps_srv.response.grasps.header.frame_id;
    if (frame == "") {
      // This must always be the camera frame passed into agile_grasp.
      frame = "/head_mount_kinect_rgb_optical_frame";
    }

    // Visualize grasp
    Point surface_center;
    surface_center.x = grasp.surface_center.x;
    surface_center.y = grasp.surface_center.y;
    surface_center.z = grasp.surface_center.z;

    rapid::viz::Marker m = rapid::viz::Marker::Vector(
        marker_pub_, frame, surface_center, grasp.approach);
    m.SetNamespace("grasp_approach");
    m.SetColor(1, 0, 0, 0.5);
    m.Publish();

    m = rapid::viz::Marker::Vector(marker_pub_, frame, surface_center,
                                   grasp.axis);
    m.SetNamespace("grasp_axis");
    m.SetColor(0, 1, 0, 0.5);
    m.Publish();

    PoseStamped grasp_pose;
    grasp_pose.header.frame_id = frame;
    grasp_pose.pose.position.x = grasp.surface_center.x;
    grasp_pose.pose.position.y = grasp.surface_center.y;
    grasp_pose.pose.position.z = grasp.surface_center.z;
    ComputeGraspOrientation(grasp.approach, grasp.axis,
                            &grasp_pose.pose.orientation);

    // Pre-grasp
    ROS_INFO("Attempting pre-grasp");
    geometry_msgs::PoseStamped pre_grasp = grasp_pose;
    double pregrasp_distance = 0;
    ros::param::param<double>("pregrasp_distance", pregrasp_distance, 0.3);
    double scale_factor =
        pregrasp_distance / rapid::utils::Norm(grasp.approach);
    pre_grasp.pose.position.x -= scale_factor * grasp.approach.x;
    pre_grasp.pose.position.y -= scale_factor * grasp.approach.y;
    pre_grasp.pose.position.z -= scale_factor * grasp.approach.z;
    success = arm_->MoveToPoseGoal(pre_grasp);
    if (!success) {
      ROS_ERROR("Failed to move to pre-grasp position.");
      continue;
    }
    gripper_->Open();

    // Grasp
    ROS_INFO("Attempting grasp");
    geometry_msgs::PoseStamped final_grasp_pose = pre_grasp;
    double grasp_distance = 0;  // Distance from the object during the grasp.
    ros::param::param<double>("grasp_distance", grasp_distance, 0.13);
    scale_factor = (pregrasp_distance - grasp_distance) /
                   rapid::utils::Norm(grasp.approach);
    final_grasp_pose.pose.position.x += scale_factor * grasp.approach.x;
    final_grasp_pose.pose.position.y += scale_factor * grasp.approach.y;
    final_grasp_pose.pose.position.z += scale_factor * grasp.approach.z;
    success = arm_->MoveToPoseGoal(final_grasp_pose);
    if (!success) {
      ROS_ERROR("Failed to execute grasp.");
      continue;
    }
    gripper_->Close();
    grasped_object = true;
    break;
  }

  if (!grasped_object) {
    ROS_ERROR("Failed to grasp object.");
    return false;
  }

  ROS_INFO("Pick succeeded");
  return true;
}

Placer::Placer(ArmInterface* arm, GripperInterface* gripper)
    : nh_(),
      marker_pub_(nh_.advertise<visualization_msgs::Marker>(
          "visualization_marker", 10)),
      arm_(arm),
      gripper_(gripper) {}

bool Placer::Place(const ScenePrimitive& obj, const Tabletop& table) {
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

void ComputeGraspOrientation(const geometry_msgs::Vector3& approach,
                             const geometry_msgs::Vector3& axis,
                             geometry_msgs::Quaternion* orientation) {
  // Not sure if grasp.axis and grasp.approach are always orthogonal, so
  // compute y = x cross z, then z = x cross y.
  Eigen::Vector3d e_approach;
  e_approach << approach.x, approach.y, approach.z;
  Eigen::Vector3d e_axis;
  e_axis << axis.x, axis.y, axis.z;
  Eigen::Vector3d grasp_y;
  grasp_y = e_approach.cross(e_axis);
  e_axis = e_approach.cross(grasp_y);

  Eigen::Matrix3d rotation;
  rotation.col(0) = e_approach;
  rotation.col(1) = grasp_y;
  rotation.col(2) = e_axis;

  Eigen::Quaterniond eigen_q(rotation);
  eigen_q.normalize();
  geometry_msgs::Quaternion grasp_q;
  orientation->w = eigen_q.w();
  orientation->x = eigen_q.x();
  orientation->y = eigen_q.y();
  orientation->z = eigen_q.z();
}
}  // namespace manipulation
}  // namespace rapid
