#include <algorithm>

#include "boost/bind.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "hcr_common_markers/six_dof_controls.h"
#include "interactive_markers/interactive_marker_server.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "rapid_collision/collision_checks.h"

using geometry_msgs::Pose;
using geometry_msgs::Vector3;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::Marker;

Marker BoxMarker(const geometry_msgs::Vector3& scale) {
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale = scale;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarker MoveableBox(const std::string& name,
                              const geometry_msgs::Vector3& scale) {
  double max_scale = std::max(std::max(scale.x, scale.y), scale.z);
  InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = "base_link";
  interactive_marker.name = name;
  interactive_marker.scale = max_scale;

  InteractiveMarkerControl box_control;
  box_control.interaction_mode = InteractiveMarkerControl::NONE;
  box_control.always_visible = true;
  box_control.markers.push_back(BoxMarker(scale));

  interactive_marker.controls =
      hcr_common_markers::FreeOrientationSixDofControls();
  interactive_marker.controls.push_back(box_control);

  return interactive_marker;
}

class Demo {
 public:
  Demo();
  void Start();
  void ProcessFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

 private:
  interactive_markers::InteractiveMarkerServer server_;
};

Demo::Demo() : server_("boxes") {}

void Demo::Start() {
  geometry_msgs::Vector3 scale1;
  scale1.x = 0.05;
  scale1.y = 0.08;
  scale1.z = 0.3;
  geometry_msgs::Vector3 scale2;
  scale2.x = 0.1;
  scale2.y = 0.2;
  scale2.z = 0.05;
  InteractiveMarker box1 = MoveableBox("box1", scale1);
  InteractiveMarker box2 = MoveableBox("box2", scale2);
  server_.insert(box1);
  server_.insert(box2);
  server_.setCallback("box1", boost::bind(&Demo::ProcessFeedback, this, _1));
  server_.setCallback("box2", boost::bind(&Demo::ProcessFeedback, this, _1));
  server_.applyChanges();
}

void Demo::ProcessFeedback(
    const InteractiveMarkerFeedback::ConstPtr& feedback) {
  if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {
    InteractiveMarker box1, box2;
    server_.get("box1", box1);
    server_.get("box2", box2);
    const Pose& pose1 = box1.pose;
    const Vector3& scale1 = box1.controls[6].markers[0].scale;
    const Pose& pose2 = box2.pose;
    const Vector3& scale2 = box2.controls[6].markers[0].scale;
    if (rapid::AreObbsInCollision(pose1, scale1, pose2, scale2)) {
      ROS_INFO("INTERSECTION");
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "box_demo");
  ROS_INFO("Open RViz and add an InteractiveMarkers display for \"boxes\".");
  Demo demo;
  demo.Start();
  ros::spin();
  return 0;
}
