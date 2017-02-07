
#include "rapid_db/name_db.hpp"
#include "rapid_utils/command_line.h"
#include "rapid_viz/landmark_viz.h"
#include "rapid_viz/scene_viz.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "object_search/commands.h"
#include "object_search/experiment_commands.h"

using namespace object_search;

int main(int argc, char** argv) {
  ros::init(argc, argv, "landmarks_experiment");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  rapid::db::NameDb task_db(nh, "custom_landmarks", "tasks");
  rapid::db::NameDb label_db(nh, "custom_landmarks", "task_labels");
  rapid::db::NameDb scene_db(nh, "custom_landmarks", "scenes");
  rapid::db::NameDb scene_cloud_db(nh, "custom_landmarks", "scene_clouds");
  rapid::db::NameDb landmark_db(nh, "custom_landmarks", "landmarks");
  rapid::db::NameDb landmark_cloud_db(nh, "custom_landmarks",
                                      "landmark_clouds");
  ExperimentDbs dbs;
  dbs.task_db = &task_db;
  dbs.label_db = &label_db;
  dbs.scene_db = &scene_db;
  dbs.scene_cloud_db = &scene_cloud_db;
  dbs.landmark_db = &landmark_db;
  dbs.landmark_cloud_db = &landmark_cloud_db;

  ros::Publisher scene_pub =
      nh.advertise<sensor_msgs::PointCloud2>("scene", 1, true);
  ros::Publisher landmark_pub =
      nh.advertise<sensor_msgs::PointCloud2>("landmark", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(
      "visualization_markers", 10, true);

  ExperimentVizs vizs;
  rapid::viz::LandmarkViz landmark_viz(landmark_pub, marker_pub);
  rapid::viz::SceneViz scene_viz(scene_pub);
  vizs.landmark_viz = &landmark_viz;
  vizs.scene_viz = &scene_viz;

  ListTasks list_tasks(&task_db);
  CreateTask create_task(dbs);
  ShowTask show_task(dbs, vizs);
  DeleteTask delete_task(&task_db);
  rapid::utils::ExitCommand exit;

  rapid::utils::CommandLine cli("Landmarks experiment");
  cli.AddCommand(&list_tasks);
  cli.AddCommand(&create_task);
  cli.AddCommand(&show_task);
  cli.AddCommand(&delete_task);
  cli.AddCommand(&exit);

  while (cli.Next()) {
  }
  spinner.stop();
  return 0;
}
