#include "rapid_manipulation/place.h"

#include <vector>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_perception/hsurface.h"
#include "rapid_perception/object.h"
#include "rapid_utils/math.h"
#include "rapid_viz/markers.h"

using geometry_msgs::PointStamped;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
using rapid::perception::HSurface;
using rapid::perception::Object;
using std::string;
using std::vector;

namespace vmsgs = visualization_msgs;

namespace rapid {
namespace manipulation {

Placer::Placer(ArmInterface* arm, GripperInterface* gripper)
    : nh_(),
      marker_pub_(new rapid_ros::Publisher<vmsgs::Marker>(
          nh_.advertise<vmsgs::Marker>("visualization_marker", 10))),
      arm_(arm),
      gripper_(gripper) {}

Placer::~Placer() { delete marker_pub_; }

bool Placer::Place(const Object& obj, const HSurface& table) {
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

bool SampleRandomPlacement(const Vector3& object_scale, const HSurface& table,
                           PointStamped* location) {
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
    const vector<Object>& objects = table.objects();
    Vector3 inflated_object = object_scale;  // Create an inflated object to
                                             // account for gripper width.
    inflated_object.x += 0.05;
    inflated_object.y += 0.05;
    for (size_t obj_i = 0; obj_i < objects.size(); ++obj_i) {
      intersect = rapid::utils::AabbXYIntersect(
          transformed.point, inflated_object,
          objects[obj_i].pose().pose.position, objects[obj_i].scale());
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
