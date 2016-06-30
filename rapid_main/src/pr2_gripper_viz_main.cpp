#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "interactive_markers/interactive_marker_server.h"
#include "rapid_viz/markers.h"
#include "rapid_viz/pr2_gripper.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/InteractiveMarker.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_gripper_viz_example");

  // Gripper 1
  geometry_msgs::PoseStamped ps1;
  ps1.header.frame_id = "base_link";
  ps1.pose.orientation.w = 1;
  visualization_msgs::InteractiveMarker gripper1 =
      rapid::viz::Pr2GripperMarker("gripper1", ps1, 1);

  // Closing region of gripper 1
  ros::NodeHandle nh;
  ros::Publisher ros_pub = nh.advertise<visualization_msgs::Marker>(
      "/visualization_markers", 100, true);
  while (ros_pub.getNumSubscribers() == 0) {
    ros::spinOnce();
  }
  rapid::viz::MarkerPub* pub =
      new rapid_ros::Publisher<visualization_msgs::Marker>(ros_pub);
  geometry_msgs::PoseStamped closing_ps;
  closing_ps.header.frame_id = "base_link";
  closing_ps.pose.orientation.w = 1;
  closing_ps.pose.position.x = 0.16;
  geometry_msgs::Vector3 closing_scale;
  closing_scale.x = 0.07;
  closing_scale.y = 0.08;
  closing_scale.z = 0.02;
  rapid::viz::Marker box =
      rapid::viz::Marker::Box(pub, closing_ps, closing_scale);
  box.SetColor(0, 0, 1, 0.5);
  box.Publish();

  // Gripper 2
  geometry_msgs::PoseStamped ps2;
  ps2.header.frame_id = "base_link";
  ps2.pose.position.y = -0.3;
  ps2.pose.orientation.w = 1;
  std_msgs::ColorRGBA green;
  green.g = 1;
  green.a = 0.8;
  visualization_msgs::InteractiveMarker gripper2 =
      rapid::viz::Pr2GripperMarker("gripper2", ps2, 0, green);

  closing_ps.pose.position.y = -0.3;
  rapid::viz::Marker box2 =
      rapid::viz::Marker::Box(pub, closing_ps, closing_scale);
  box2.SetColor(0, 0, 1, 0.5);
  box2.SetNamespace("gripper2");
  box2.Publish();

  interactive_markers::InteractiveMarkerServer server("grippers");
  server.insert(gripper1);
  server.insert(gripper2);
  server.applyChanges();
  ros::spin();
}
