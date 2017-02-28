#include <iostream>
#include <string>

#include "geometry_msgs/PoseArray.h"
#include "pcl/filters/filter.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_db/name_db.hpp"
#include "rapid_msgs/LandmarkInfo.h"
#include "rapid_msgs/SceneInfo.h"
#include "rapid_msgs/GetStaticCloud.h"
#include "rapid_msgs/ListStaticClouds.h"
#include "rapid_msgs/RemoveStaticCloud.h"
#include "rapid_msgs/SaveStaticCloud.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_perception/grouping_pose_estimator.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/random_heat_mapper.h"
#include "rapid_perception/ransac_pose_estimator.h"
#include "rapid_utils/command_line.h"
#include "rapid_viz/scene_viz.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "object_search/capture_roi.h"
#include "object_search/commands.h"
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
  rapid::db::NameDb scene_ndb(nh, "custom_landmarks", "scenes");
  rapid::db::NameDb scene_cloud_ndb(nh, "custom_landmarks", "scene_clouds");
  rapid::db::NameDb landmark_ndb(nh, "custom_landmarks", "landmarks");
  rapid::db::NameDb landmark_cloud_ndb(nh, "custom_landmarks",
                                       "landmark_clouds");

  // Visualization publishers
  ros::Publisher landmark_pub = nh.advertise<PointCloud2>("/landmark", 1, true);
  ros::Publisher scene_pub = nh.advertise<PointCloud2>("/scene", 1, true);
  ros::Publisher heatmap_pub = nh.advertise<PointCloud2>("/heatmap", 1, true);
  ros::Publisher candidates_pub =
      nh.advertise<PointCloud2>("/candidate_samples", 1, true);
  ros::Publisher alignment_pub =
      nh.advertise<PointCloud2>("/alignment", 1, true);
  ros::Publisher output_pub = nh.advertise<PointCloud2>("/output", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(
      "/visualization_markers", 1, true);

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
  // pose_estimator.set_scene_publisher(scene_pub);
  pose_estimator.set_candidates_publisher(candidates_pub);
  pose_estimator.set_alignment_publisher(alignment_pub);
  // pose_estimator.set_marker_publisher(&marker_pub);

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

  // Shared state for main CLI
  PoseEstimatorInput estimator_input;

  // Build visualizers
  rapid::viz::SceneViz scene_viz(scene_pub);

  // Build commands
  ListCommand list_landmarks(&landmark_ndb, ListCommand::kLandmarks, "list",
                             "- List landmarks");
  ListCommand list_landmarks2(&landmark_ndb, ListCommand::kLandmarks,
                              "list landmarks", "- List landmarks");
  ListCommand list_scenes(&scene_ndb, ListCommand::kScenes, "list",
                          "- List scenes");
  ListCommand list_scenes2(&scene_ndb, ListCommand::kScenes, "list scenes",
                           "- List scenes");
  // ListCommand list_scenes_named(&scene_ndb, ListCommand::kScenes, "scenes",
  //                              "- List scenes");
  sensor_msgs::PointCloud2::Ptr landmark_scene(new sensor_msgs::PointCloud2);
  EditLandmarkCommand create_landmark(
      &landmark_ndb, &landmark_cloud_ndb, &scene_ndb, &scene_cloud_ndb,
      &roi_server, &scene_viz, std::string(EditLandmarkCommand::kCreate));
  EditLandmarkCommand edit_landmark(
      &landmark_ndb, &landmark_cloud_ndb, &scene_ndb, &scene_cloud_ndb,
      &roi_server, &scene_viz, std::string(EditLandmarkCommand::kEdit));
  RecordSceneCommand record_scene(&scene_ndb, &scene_cloud_ndb);
  DeleteCommand delete_landmark(&landmark_ndb, &landmark_cloud_ndb, "landmark");
  DeleteCommand delete_scene(&scene_ndb, &scene_cloud_ndb, "scene");
  SetInputLandmarkCommand set_input_landmark(&landmark_ndb, &landmark_cloud_ndb,
                                             landmark_pub, marker_pub, &estimator_input);
  SetInputSceneCommand set_input_scene(&scene_ndb, &scene_cloud_ndb, scene_viz,
                                       &estimator_input);
  // SetLandmarkSceneCommand set_landmark_scene(&scene_ndb, &estimator_input,
  //                                           "scene");
  RunCommand run(&estimators, &estimator_input, output_pub, marker_pub);
  SetDebugCommand set_debug(&estimators);
  rapid::utils::ExitCommand exit;

  ShowSceneCommand show_scene(&scene_cloud_ndb, &scene_viz);

  // Build CLIs
  // Scene manager
  rapid::utils::CommandLine scene_cli("Scene manager");
  scene_cli.AddCommand(&list_scenes);
  scene_cli.AddCommand(&record_scene);
  scene_cli.AddCommand(&show_scene);
  scene_cli.AddCommand(&delete_scene);
  scene_cli.AddCommand(&exit);

  // Landmark editor
  // rapid::utils::CommandLine landmark_cli("Landmark editor");
  // landmark_cli.AddCommand(&list_scenes_named);
  // landmark_cli.AddCommand(&set_landmark_scene);
  // landmark_cli.AddCommand(&exit);

  // Landmark manager
  rapid::utils::CommandLine landmarks_cli("Landmark manager");
  landmarks_cli.AddCommand(&list_landmarks);
  landmarks_cli.AddCommand(&create_landmark);
  landmarks_cli.AddCommand(&edit_landmark);
  landmarks_cli.AddCommand(&delete_landmark);
  landmarks_cli.AddCommand(&exit);

  // Main CLI
  CliCommand edit_scenes(scene_cli, "edit scenes", "- Edit scenes");
  CliCommand edit_landmarks(landmarks_cli, "edit landmarks",
                            "- Edit landmarks");

  rapid::utils::CommandLine cli("Custom landmarks CLI");
  cli.AddCommand(&list_scenes2);
  cli.AddCommand(&list_landmarks2);
  cli.AddCommand(&edit_scenes);
  cli.AddCommand(&edit_landmarks);
  cli.AddCommand(&set_input_landmark);
  cli.AddCommand(&set_input_scene);
  cli.AddCommand(&run);
  cli.AddCommand(&set_debug);
  cli.AddCommand(&exit);

  while (cli.Next()) {
  }

  scene_viz.Clear();

  spinner.stop();
  return 0;
}
