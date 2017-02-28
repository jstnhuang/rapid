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
                const PoseEstimationMatch& match,
                const std::string& landmark_name);
bool IsNegativeLandmark(const std::vector<object_search_msgs::Label>& labels,
                        const std::string& landmark_name);

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
  ros::Publisher marker_pub =
      nh.advertise<Marker>("/visualization_markers", 1, true);

  rapid::perception::RandomHeatMapper heat_mapper;
  heat_mapper.set_name("random");
  heat_mapper.set_heatmap_publisher(heatmap_pub);
  rapid::perception::PoseEstimator custom(&heat_mapper);
  custom.set_candidates_publisher(candidates_pub);
  custom.set_alignment_publisher(alignment_pub);
  custom.set_marker_publisher(marker_pub);
  custom.set_output_publisher(output_pub);
  custom.set_scene_publisher(scene_pub);

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
  std::vector<object_search::ConfusionMatrix> task_confusions;
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
      if (IsNegativeLandmark(task.labels, *landmark_name)) {
        // If this landmark is not supposed to be in the scene, penalize the
        // false positive.
        landmark_results.fp += matches.size();
      } else {
        // If the landmark is supposed to be in the scene, check the location of
        // the matches. Based on the results, we can count false positives,
        // false negatives, and true positives.
        std::vector<int> found(task.labels.size(), 0);
        for (size_t mi = 0; mi < matches.size(); ++mi) {
          const PoseEstimationMatch& match = matches[mi];
          int label_i = MatchLabels(task.labels, match, *landmark_name);
          if (label_i == -1) {
            ROS_INFO("Match %ld was not found.", mi);
            ++landmark_results.fp;
          } else if (task.labels[label_i].exists) {
            ROS_INFO("Match %ld corresponds to %d.", mi, label_i);
            found[label_i] = 1;
          }
        }

        // Count true positives and false negatives (iterate over labels)
        for (size_t fi = 0; fi < found.size(); ++fi) {
          if (task.labels[fi].landmark_name != *landmark_name) {
            continue;
          }
          if (!task.labels[fi].exists) {
            continue;
          }
          if (found[fi] == 1) {
            ++landmark_results.tp;
          } else {
            ++landmark_results.fn;
          }
        }
      }
      task_results.Merge(landmark_results);
    }
    task_confusions.push_back(task_results);
    results.Merge(task_results);
  }
  for (size_t i = 0; i < task_confusions.size(); ++i) {
    const std::string& task_name = task_list[i];
    const object_search::ConfusionMatrix& task_results = task_confusions[i];
    std::cout << " Task \"" << task_name << "\":"
              << " Precision: " << task_results.Precision()
              << ", Recall: " << task_results.Recall()
              << ", F1: " << task_results.F1() << std::endl;
    std::cout << " tp: " << task_results.tp << ", fp: " << task_results.fp
              << ", tn: " << task_results.tn << ", fn: " << task_results.fn
              << std::endl;
  }

  std::cout << "Final results:" << std::endl;
  std::cout << "Precision: " << results.Precision()
            << ", Recall: " << results.Recall() << ", F1: " << results.F1()
            << std::endl;
  std::cout << "tp: " << results.tp << ", fp: " << results.fp
            << ", tn: " << results.tn << ", fn: " << results.fn << std::endl;
}

int MatchLabels(const std::vector<object_search_msgs::Label>& labels,
                const PoseEstimationMatch& match,
                const std::string& landmark_name) {
  bool debug = false;
  ros::param::param<bool>("experiment_debug", debug, false);

  double position_tolerance;
  double orientation_tolerance;  // Approx 2 degrees
  ros::param::param<double>("position_tolerance", position_tolerance, 0.0254);
  ros::param::param<double>("orientation_tolerance", orientation_tolerance,
                            0.04);
  for (size_t i = 0; i < labels.size(); ++i) {
    const object_search_msgs::Label& label = labels[i];
    if (!label.exists) {
      continue;
    }
    if (landmark_name != label.landmark_name) {
      continue;
    }
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

    if (debug) {
      ROS_INFO("pos diff: %f, ang diff: %f", pos_diff, ang_diff);
    }
  }
  return -1;
}

bool IsNegativeLandmark(const std::vector<object_search_msgs::Label>& labels,
                        const std::string& landmark_name) {
  for (size_t i = 0; i < labels.size(); ++i) {
    const object_search_msgs::Label& label = labels[i];
    if (label.landmark_name == landmark_name && label.exists) {
      return false;
    } else if (label.landmark_name == landmark_name && !label.exists) {
      return true;
    }
  }
  return false;
}
