#include <iostream>
#include <string>
#include <vector>

#include "pcl/common/time.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/sample_consensus_prerejective.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_perception/pr2.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_perception/image_recognition.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_msgs/GetStaticCloud.h"
#include "rapid_msgs/ListStaticClouds.h"
#include "rapid_msgs/RemoveStaticCloud.h"
#include "rapid_msgs/SaveStaticCloud.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "object_search/capture_roi.h"
#include "object_search/commands.h"
#include "object_search/command_line.h"
#include "object_search/cloud_database.h"

using sensor_msgs::Image;
using sensor_msgs::PointCloud2;
using pcl::FPFHSignature33;
using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::PointXYZRGBNormal;
using visualization_msgs::Marker;
namespace rp = rapid::perception;

using namespace object_search;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "object_search_main /path/to/alexnet" << std::endl;
    return 1;
  }
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
  ros::Publisher landmark_image_pub =
      nh.advertise<Image>("/landmark_image", 1, true);
  ros::Publisher scene_image_pub = nh.advertise<Image>("/scene_image", 1, true);

  // Build ROI server
  tf::TransformListener tf_listener;
  rapid::perception::Box3DRoiServer roi_server("roi");
  CaptureRoi capture(&roi_server);

  // Build image recognizer
  rapid::perception::ImageRecognizer image_recognizer;
  std::string error;
  rapid::perception::ImageRecognizer::AlexNet(argv[1], &image_recognizer,
                                              &error);

  // Build heat mapper
  std::string heat_mapper_type = "cnn";
  rapid::perception::CnnHeatMapper* cnn_heat_mapper =
      new rapid::perception::CnnHeatMapper();
  cnn_heat_mapper->set_image_recognizer(image_recognizer);
  cnn_heat_mapper->set_heatmap_publisher(heatmap_pub);
  cnn_heat_mapper->set_landmark_image_publisher(landmark_image_pub);
  cnn_heat_mapper->set_scene_image_publisher(scene_image_pub);

  // Build pose estimator
  rapid::perception::PoseEstimator pose_estimator;
  pose_estimator.set_heat_mapper(cnn_heat_mapper);

  ListCommand list_objects(&object_db, "object");
  ListCommand list_scenes(&scene_db, "scene");
  RecordObjectCommand record_object(&object_db, &capture);
  RecordSceneCommand record_scene(&scene_db);
  DeleteCommand delete_object(&object_db);
  DeleteCommand delete_scene(&scene_db);
  UseCommand use_object(&object_db, &pose_estimator, "object", object_pub,
                        heat_mapper_type);
  UseCommand use_scene(&scene_db, &pose_estimator, "scene", scene_pub,
                       heat_mapper_type);
  RunCommand run(&pose_estimator, candidates_pub, alignment_pub, output_pub,
                 heat_mapper_type);
  SetDebugCommand set_debug(&pose_estimator);

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
