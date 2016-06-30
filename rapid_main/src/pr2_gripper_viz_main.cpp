#include "geometry_msgs/PoseStamped.h"
#include "interactive_markers/interactive_marker_server.h"
#include "rapid_viz/pr2_gripper.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/InteractiveMarker.h"

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM(feedback->marker_name << " is now at "
                                        << feedback->pose.position.x << ", "
                                        << feedback->pose.position.y << ", "
                                        << feedback->pose.position.z);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_gripper_viz_example");

  // Gripper 1
  geometry_msgs::PoseStamped ps1;
  ps1.header.frame_id = "base_link";
  ps1.pose.orientation.w = 1;
  visualization_msgs::InteractiveMarker gripper1 =
      rapid::viz::Pr2GripperMarker("gripper1", ps1, 1);

  // Gripper 2
  geometry_msgs::PoseStamped ps2;
  ps2.header.frame_id = "base_link";
  ps2.pose.position.y = -0.3;
  ps2.pose.orientation.w = 1;
  std_msgs::ColorRGBA green;
  green.g = 1;
  green.a = 1;
  visualization_msgs::InteractiveMarker gripper2 =
      rapid::viz::Pr2GripperMarker("gripper2", ps2, 0, green);

  interactive_markers::InteractiveMarkerServer server("grippers");
  server.insert(gripper1, &processFeedback);
  server.insert(gripper2);
  server.applyChanges();
  ros::spin();
}
