#include "rapid_manipulation/place.h"

#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_perception/hsurface.h"
#include "rapid_perception/object.h"
#include "rapid_utils/math.h"
#include "rapid_viz/markers.h"

using geometry_msgs::Point;
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
Placer::Placer(ArmInterface* arm, GripperInterface* gripper,
               viz::MarkerPub* marker_pub)
    : marker_pub_(marker_pub), arm_(arm), gripper_(gripper) {}

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
    PointStamped loc_position;
    SampleRandomPlacement(obj.scale().z, table, &loc_position);
    PoseStamped location;
    location.header = loc_position.header;
    location.pose.position = loc_position.point;
    location.pose.orientation.w = 1;

    VisualizePlacement(location, obj.scale());

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

void Placer::VisualizePlacement(const PoseStamped& obj_ps,
                                const Vector3& obj_scale) {
  rapid::viz::Marker marker =
      rapid::viz::Marker::Box(marker_pub_, obj_ps, obj_scale);
  marker.SetNamespace("sampled_placement");
  marker.SetColor(1, 1, 0, 0.5);
  marker.Publish();
}

bool SampleRandomPlacement(double obj_height, const HSurface& table,
                           PointStamped* location) {
  double distance_from_edge = 0;
  ros::param::param<double>("distance_from_edge", distance_from_edge, 0.05);
  geometry_msgs::PoseStamped table_ps = table.pose();
  double x_range = table.scale().x - 2 * distance_from_edge;
  double start_x = -table.scale().x / 2 + distance_from_edge;
  double y_range = table.scale().y - 2 * distance_from_edge;
  double start_y = -table.scale().y / 2 + distance_from_edge;
  double z = table.scale().z / 2 + obj_height / 2;

  tf::Transform table_frame;  // Transform from table to base
  tf::Vector3 table_origin(table_ps.pose.position.x, table_ps.pose.position.y,
                           table_ps.pose.position.z);
  tf::Quaternion table_orientation(
      table_ps.pose.orientation.x, table_ps.pose.orientation.y,
      table_ps.pose.orientation.z, table_ps.pose.orientation.w);
  table_frame.setOrigin(table_origin);
  table_frame.setRotation(table_orientation);

  int max_tries = 0;
  ros::param::param<int>("sample_placement_max_tries", max_tries, 100);
  double obstacle_distance = 0;
  ros::param::param<double>("obstacle_distance", obstacle_distance, 0.1);
  double obs_dist_squared = obstacle_distance * obstacle_distance;
  for (int i = 0; i < max_tries; ++i) {
    // Sample in table space
    tf::Vector3 pos;
    pos.setX(start_x + static_cast<double>(std::rand()) / RAND_MAX * x_range);
    pos.setY(start_y + static_cast<double>(std::rand()) / RAND_MAX * y_range);
    pos.setZ(z);
    tf::Vector3 transformed = table_frame * pos;
    ROS_INFO("Try %d (base frame): x=%f, y=%f, z=%f", i, transformed.x(),
             transformed.y(), transformed.z());

    // Check that the object doesn't intersect with all other objects.
    // TODO(jstn): do a real check of object intersection. For now, just check
    // that it's some distance away from other objects.
    // TODO(jstn): this assumes objects are in the base frame.
    bool intersect = false;
    const vector<Object>& objects = table.objects();
    for (size_t obj_i = 0; obj_i < objects.size(); ++obj_i) {
      const Point& obj_pos = objects[obj_i].pose().pose.position;
      double xd = obj_pos.x - transformed.x();
      double yd = obj_pos.y - transformed.y();
      double squared_distance = xd * xd + yd * yd;
      intersect = squared_distance < obs_dist_squared;
      if (intersect) {
        break;
      }
    }
    if (intersect) {
      continue;
    }

    location->header.frame_id = table.pose().header.frame_id;
    location->point.x = transformed.x();
    location->point.y = transformed.y();
    location->point.z = transformed.z();

    return true;
  }
  return false;
}
}  // namespace manipulation
}  // namespace rapid
