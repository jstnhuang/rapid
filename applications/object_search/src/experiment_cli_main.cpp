
#include "rapid_db/name_db.hpp"
#include "rapid_utils/command_line.h"
#include "rapid_viz/landmark_viz.h"
#include "rapid_viz/scene_viz.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "object_search/experiment_commands.h"
#include "object_search_msgs/Task.h"

using namespace object_search;

int main(int argc, char** argv) {
  ros::init(argc, argv, "landmarks_experiment");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Databases
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

  // Visualizers
  ros::Publisher scene_pub =
      nh.advertise<sensor_msgs::PointCloud2>("scene", 1, true);
  ros::Publisher landmark_pub =
      nh.advertise<sensor_msgs::PointCloud2>("landmark", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(
      "visualization_markers", 10, true);
  TaskViz task_viz(dbs, scene_pub, landmark_pub, marker_pub);

  // Task editor commands and CLI
  object_search_msgs::Task task;    // Temporary space for task editor.
  object_search_msgs::Label label;  // Temporary space for label editing.
  ListLandmarksOrScenes list_scenes(&scene_db, ListLandmarksOrScenes::kScenes,
                                    "scenes", "- List scenes");
  ListLandmarksOrScenes list_landmarks(&landmark_db,
                                       ListLandmarksOrScenes::kLandmarks,
                                       "landmarks", "- List landmarks");
  SetTaskScene set_task_scene(dbs, &task);
  SetLabelLandmark set_label_landmark(dbs, &task, &label);
  AddLabel add_label(dbs, landmark_pub, marker_pub, &task, &label);
  AddNegativeLabel add_negative_label(dbs, &task);
  ListLabels list_labels(&task);
  DeleteLabel delete_label(dbs, &task);
  rapid::utils::ExitCommand exit;

  rapid::utils::CommandLine task_cli("Task editor");
  task_cli.AddCommand(&list_scenes);
  task_cli.AddCommand(&list_landmarks);
  task_cli.AddCommand(&set_task_scene);
  task_cli.AddCommand(&set_label_landmark);
  task_cli.AddCommand(&add_label);
  task_cli.AddCommand(&add_negative_label);
  task_cli.AddCommand(&list_labels);
  task_cli.AddCommand(&delete_label);
  task_cli.AddCommand(&exit);

  // Main commands and CLI
  ListTasks list_tasks(&task_db);
  CreateTask create_task(dbs);
  EditTask edit_task(dbs, task_viz, &task_cli, &task);
  DeleteTask delete_task(&task_db);

  rapid::utils::CommandLine cli("Landmarks experiment");
  cli.AddCommand(&list_tasks);
  cli.AddCommand(&create_task);
  cli.AddCommand(&edit_task);
  cli.AddCommand(&delete_task);
  cli.AddCommand(&exit);

  while (cli.Next()) {
  }

  task_viz.Clear();
  spinner.stop();
  return 0;
}
