#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "object_search_msgs/Label.h"
#include "object_search_msgs/Task.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_db/name_db.hpp"
#include "rapid_msgs/LandmarkInfo.h"
#include "rapid_perception/conversions.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/random_heat_mapper.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "object_search/experiment.h"
#include "object_search/object_search.h"

using object_search::ExperimentDbs;
using rapid::perception::PoseEstimationMatch;
using rapid::perception::PoseEstimator;
using sensor_msgs::PointCloud2;
using visualization_msgs::Marker;
using pcl::PointCloud;
using pcl::PointXYZRGB;

typedef PointCloud<PointXYZRGB> PointC;

void RunExperiment(PoseEstimator* estimator,
                   const std::vector<std::string>& task_list,
                   const ExperimentDbs& dbs);

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_search_experiment");
  ros::NodeHandle nh;

  // Build DBs
  rapid::db::NameDb task_db(nh, "custom_landmarks", "tasks");
  rapid::db::NameDb scene_db(nh, "custom_landmarks", "scenes");
  rapid::db::NameDb scene_cloud_db(nh, "custom_landmarks", "scene_clouds");
  rapid::db::NameDb landmark_db(nh, "custom_landmarks", "landmarks");
  rapid::db::NameDb landmark_cloud_db(nh, "custom_landmarks",
                                      "landmark_clouds");
  ExperimentDbs dbs;
  dbs.task_db = &task_db;
  dbs.scene_db = &scene_db;
  dbs.scene_cloud_db = &scene_cloud_db;
  dbs.landmark_db = &landmark_db;
  dbs.landmark_cloud_db = &landmark_cloud_db;

  // Build estimator
  // Visualization publishers
  ros::Publisher landmark_pub = nh.advertise<PointCloud2>("/landmark", 1, true);
  ros::Publisher scene_pub = nh.advertise<PointCloud2>("/scene", 1, true);
  ros::Publisher heatmap_pub = nh.advertise<PointCloud2>("/heatmap", 1, true);
  ros::Publisher candidates_pub =
      nh.advertise<PointCloud2>("/candidate_samples", 1, true);
  ros::Publisher alignment_pub =
      nh.advertise<PointCloud2>("/alignment", 1, true);
  ros::Publisher output_pub = nh.advertise<PointCloud2>("/output", 1, true);
  rapid_ros::Publisher<Marker> marker_pub(
      nh.advertise<Marker>("/visualization_markers", 1, true));

  rapid::perception::RandomHeatMapper heat_mapper;
  heat_mapper.set_name("random");
  heat_mapper.set_heatmap_publisher(heatmap_pub);
  rapid::perception::PoseEstimator custom(&heat_mapper);
  custom.set_candidates_publisher(candidates_pub);
  custom.set_alignment_publisher(alignment_pub);
  custom.set_marker_publisher(&marker_pub);

  // Read tasks from the parameter server
  std::vector<std::string> task_list;
  nh.getParam("experiment_tasks", task_list);

  RunExperiment(&custom, task_list, dbs);

  ros::spin();
  return 0;
}

void RunExperiment(PoseEstimator* estimator,
                   const std::vector<std::string>& task_list,
                   const ExperimentDbs& dbs) {
  for (size_t task_i = 0; task_i < task_list.size(); ++task_i) {
    const std::string& task_name = task_list[task_i];
    object_search_msgs::Task task;
    if (!dbs.task_db->Get(task_name, &task)) {
      std::cerr << "Error getting task \"" << task_name << "\", skipping."
                << std::endl;
      continue;
    }

    sensor_msgs::PointCloud2 scene_cloud;
    if (!dbs.scene_cloud_db->Get(task.scene_name, &scene_cloud)) {
      std::cerr << "Error getting scene cloud \"" << task.scene_name
                << "\" for task \"" << task_name << "\", skipping."
                << std::endl;
      continue;
    }

    std::set<std::string> landmarks;
    for (size_t li = 0; li < task.labels.size(); ++li) {
      const object_search_msgs::Label& label = task.labels[li];
      landmarks.insert(label.landmark_name);
    }

    for (std::set<std::string>::iterator landmark_name = landmarks.begin();
         landmark_name != landmarks.end(); ++landmark_name) {
      rapid_msgs::LandmarkInfo landmark_info;
      if (!dbs.landmark_db->Get(*landmark_name, &landmark_info)) {
        std::cerr << "Error getting landmark info \"" << *landmark_name
                  << "\" for task \"" << task_name << "\", skipping."
                  << std::endl;
        continue;
      }

      sensor_msgs::PointCloud2 landmark_cloud;
      if (!dbs.landmark_cloud_db->Get(*landmark_name, &landmark_cloud)) {
        std::cerr << "Error getting landmark cloud \"" << *landmark_name
                  << "\" for task \"" << task_name << "\", skipping."
                  << std::endl;
        continue;
      }

      object_search::UpdateEstimatorParams(estimator);

      PointC::Ptr scene = rapid::perception::PclFromRos(scene_cloud);
      estimator->set_scene(scene);
      PointC::Ptr landmark = rapid::perception::PclFromRos(landmark_cloud);
      estimator->set_object(landmark);
      estimator->set_roi(landmark_info.roi);

      std::vector<rapid::perception::PoseEstimationMatch> matches;
      estimator->Find(&matches);

      // Compute precision/recall
      // TODO: complete this
    }
  }
}
