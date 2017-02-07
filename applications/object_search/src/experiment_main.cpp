#include "ros/ros.h"

#include "rapid_db/name_db.hpp"
#include "rapid_utils/command_line.h"

#include "object_search/commands.h"
#include "object_search/experiment_commands.h"

using namespace object_search;

int main(int argc, char** argv) {
  ros::init(argc, argv, "landmarks_experiment");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  rapid::db::NameDb task_db(nh, "custom_landmarks", "tasks");

  rapid::utils::ExitCommand exit;
  ListTasks list_tasks(&task_db);

  rapid::utils::CommandLine cli("Landmarks experiment");
  cli.AddCommand(&list_tasks);
  cli.AddCommand(&exit);

  while (cli.Next()) {
  }
  spinner.stop();
  return 0;
}
