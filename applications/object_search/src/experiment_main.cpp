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
int MatchLabels(const std::vector<object_search_msgs::Label>& labels,
                const PoseEstimationMatch& match);

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

  return 0;
}

void RunExperiment(PoseEstimator* estimator,
                   const std::vector<std::string>& task_list,
                   const ExperimentDbs& dbs) {
  object_search::ConfusionMatrix results;
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

    object_search::ConfusionMatrix task_results;
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

      double leaf_size = 0.01;
      ros::param::param<double>("leaf_size", leaf_size, 0.01);
      PointC::Ptr scene = rapid::perception::PclFromRos(scene_cloud);
      PointC::Ptr scene_cropped(new PointC);
      object_search::CropScene(scene, scene_cropped);
      PointC::Ptr scene_downsampled(new PointC);
      object_search::Downsample(leaf_size, scene_cropped, scene_downsampled);
      estimator->set_scene(scene_downsampled);

      PointC::Ptr landmark = rapid::perception::PclFromRos(landmark_cloud);
      PointC::Ptr landmark_downsampled(new PointC);
      object_search::Downsample(leaf_size, landmark, landmark_downsampled);
      estimator->set_object(landmark_downsampled);
      estimator->set_roi(landmark_info.roi);

      std::vector<PoseEstimationMatch> matches;
      estimator->Find(&matches);

      // Compute precision/recall
      object_search::ConfusionMatrix landmark_results;
      std::vector<int> found(task.labels.size(), 0);
      for (size_t mi = 0; mi < matches.size(); ++mi) {
        const PoseEstimationMatch& match = matches[mi];
        int label_i = MatchLabels(task.labels, match);
        if (label_i != -1) {
          found[label_i] = 1;
        } else {
          ++landmark_results.fp;
        }
      }

      for (size_t fi = 0; fi < found.size(); ++fi) {
        if (found[fi] == 1) {
          ++landmark_results.tp;
        } else {
          ++landmark_results.fn;
        }
      }
      task_results.Merge(landmark_results);
      std::cout << "  Results for task \"" << task_name << "\", landmark \""
                << *landmark_name << "\"" << std::endl;
      std::cout << " Precision: " << landmark_results.Precision()
                << ", Recall: " << landmark_results.Recall()
                << ", F1: " << landmark_results.F1() << std::endl;
    }
    results.Merge(task_results);
    std::cout << " Task results for \"" << task_name << "\"" << std::endl;
    std::cout << " Precision: " << task_results.Precision()
              << ", Recall: " << task_results.Recall()
              << ", F1: " << task_results.F1() << std::endl;
  }

  std::cout << "Final results:" << std::endl;
  std::cout << "Precision: " << results.Precision()
            << ", Recall: " << results.Recall() << ", F1: " << results.F1()
            << std::endl;
}

int MatchLabels(const std::vector<object_search_msgs::Label>& labels,
                const PoseEstimationMatch& match) {
  const double position_tolerance = 0.01;
  const double orientation_tolerance = 0.04;  // Approx 2 degrees
  for (size_t i = 0; i < labels.size(); ++i) {
    const object_search_msgs::Label& label = labels[i];
    double x_diff = label.pose.position.x - match.pose().position.x;
    double y_diff = label.pose.position.y - match.pose().position.y;
    double z_diff = label.pose.position.z - match.pose().position.z;
    double pos_diff =
        sqrt((x_diff * x_diff) + (y_diff * y_diff) + (z_diff * z_diff));
    Eigen::Quaterniond label_q;
    label_q.w() = label.pose.orientation.w;
    label_q.x() = label.pose.orientation.x;
    label_q.y() = label.pose.orientation.y;
    label_q.z() = label.pose.orientation.z;
    Eigen::Quaterniond match_q;
    match_q.w() = match.pose().orientation.w;
    match_q.x() = match.pose().orientation.x;
    match_q.y() = match.pose().orientation.y;
    match_q.z() = match.pose().orientation.z;
    double ang_diff = label_q.angularDistance(match_q);
    if (pos_diff < position_tolerance && ang_diff < orientation_tolerance) {
      return i;
    }
  }
  return -1;
}
