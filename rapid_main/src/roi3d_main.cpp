#include "ros/ros.h"
#include "rapid_perception/box3d_roi_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "roi3d_main");
  rapid::perception::Box3DRoiServer server("roi1");
  server.Start();
  ros::spin();
  return 0;
}
