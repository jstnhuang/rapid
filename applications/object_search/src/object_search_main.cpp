#include <iostream>
#include <string>

#include "geometry_msgs/PoseArray.h"
#include "pcl/filters/filter.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/random_heat_mapper.h"
#include "rapid_perception/ransac_pose_estimator.h"
#include "rapid_perception/grouping_pose_estimator.h"
#include "rapid_msgs/GetStaticCloud.h"
#include "rapid_msgs/ListStaticClouds.h"
#include "rapid_msgs/RemoveStaticCloud.h"
#include "rapid_msgs/SaveStaticCloud.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "object_search/capture_roi.h"
#include "object_search/commands.h"
#include "object_search/command_line.h"
#include "object_search/cloud_database.h"
#include "object_search/estimators.h"

using sensor_msgs::PointCloud2;
using pcl::PointCloud;
using pcl::PointXYZRGB;
using visualization_msgs::Marker;

using namespace object_search;

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_search");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Build databases
  ros::ServiceClient get_cloud =
      nh.serviceClient<rapid_msgs::GetStaticCloud>("get_static_cloud");
  ros::ServiceClient list_clouds =
      nh.serviceClient<rapid_msgs::ListStaticClouds>("list_static_clouds");
  ros::ServiceClient remove_cloud =
      nh.serviceClient<rapid_msgs::RemoveStaticCloud>("remove_static_cloud");
  ros::ServiceClient save_cloud =
      nh.serviceClient<rapid_msgs::SaveStaticCloud>("save_static_cloud");
  Database object_db("object_search", "objects", get_cloud, list_clouds,
                     remove_cloud, save_cloud);
  Database scene_db("object_search", "scenes", get_cloud, list_clouds,
                    remove_cloud, save_cloud);

  // Visualization publishers
  ros::Publisher object_pub = nh.advertise<PointCloud2>("/landmark", 1, true);
  ros::Publisher scene_pub = nh.advertise<PointCloud2>("/scene", 1, true);
  ros::Publisher heatmap_pub = nh.advertise<PointCloud2>("/heatmap", 1, true);
  ros::Publisher candidates_pub =
      nh.advertise<PointCloud2>("/candidate_samples", 1, true);
  ros::Publisher alignment_pub =
      nh.advertise<PointCloud2>("/alignment", 1, true);
  ros::Publisher output_pub = nh.advertise<PointCloud2>("/output", 1, true);
  rapid_ros::Publisher<visualization_msgs::Marker> marker_pub(
      nh.advertise<visualization_msgs::Marker>("/visualization_markers", 1,
                                               true));

  // Build ROI server
  rapid::perception::Box3DRoiServer roi_server("roi");
  CaptureRoi capture(&roi_server);

  // Build heat mapper
  // rapid::perception::FpfhHeatMapper* heat_mapper =
  //    new rapid::perception::FpfhHeatMapper();
  // heat_mapper->set_name("fpfh");

  // rapid::perception::TemplateMatchingHeatMapper* heat_mapper =
  //    new rapid::perception::TemplateMatchingHeatMapper();
  // heat_mapper->set_name("template_matching");

  rapid::perception::RandomHeatMapper* heat_mapper =
      new rapid::perception::RandomHeatMapper();
  heat_mapper->set_name("random");
  heat_mapper->set_heatmap_publisher(heatmap_pub);

  // Build pose estimator
  rapid::perception::PoseEstimator pose_estimator(heat_mapper);
  pose_estimator.set_candidates_publisher(candidates_pub);
  pose_estimator.set_alignment_publisher(alignment_pub);
  pose_estimator.set_marker_publisher(&marker_pub);

  rapid::perception::RansacPoseEstimator ransac_estimator;
  ros::Publisher pose_pub =
      nh.advertise<geometry_msgs::PoseArray>("/poses", 1, true);
  ransac_estimator.set_pose_publisher(pose_pub);

  rapid::perception::GroupingPoseEstimator grouping_estimator;
  ros::Publisher corr_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "/correspondences", 1, true);
  ros::Publisher object_keypoints_pub =
      nh.advertise<PointCloud2>("/landmark_keypoints", 1, true);
  ros::Publisher scene_keypoints_pub =
      nh.advertise<PointCloud2>("/scene_keypoints", 1, true);
  grouping_estimator.set_correspondence_publisher(corr_pub);
  grouping_estimator.set_scene_keypoints_publisher(scene_keypoints_pub);
  grouping_estimator.set_object_keypoints_publisher(object_keypoints_pub);

  Estimators estimators;
  estimators.custom = &pose_estimator;
  estimators.ransac = &ransac_estimator;
  estimators.grouping = &grouping_estimator;

  ListCommand list_objects(&object_db, "object");
  ListCommand list_scenes(&scene_db, "scene");
  RecordObjectCommand record_object(&object_db, &capture);
  RecordSceneCommand record_scene(&scene_db);
  DeleteCommand delete_object(&object_db);
  DeleteCommand delete_scene(&scene_db);
  UseCommand use_object(&object_db, &estimators, "object", object_pub);
  UseCommand use_scene(&scene_db, &estimators, "scene", scene_pub);
  RunCommand run(&estimators, output_pub);
  SetDebugCommand set_debug(&estimators);

  CommandLine cli;
  cli.set_list_objects(&list_objects);
  cli.set_list_scenes(&list_scenes);
  cli.set_record_object(&record_object);
  cli.set_record_scene(&record_scene);
  cli.set_delete_object(&delete_object);
  cli.set_delete_scene(&delete_scene);
  cli.set_use_object(&use_object);
  cli.set_use_scene(&use_scene);
  cli.set_run(&run);
  cli.set_debug(&set_debug);

  while (cli.Next()) {
  }

  spinner.stop();
  return 0;
}
