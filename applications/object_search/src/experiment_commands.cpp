#include "object_search/experiment_commands.h"

#include <iostream>

#include "boost/algorithm/string.hpp"
#include "object_search_msgs/Task.h"

namespace object_search {
ListTasks::ListTasks(rapid::db::NameDb* db) : db_(db) {}
void ListTasks::Execute(const std::vector<std::string>& args) {
  std::vector<std::string> names;
  db_->List<object_search_msgs::Task>(&names);

  if (names.size() == 0) {
    std::cout << "None" << std::endl;
  } else {
    for (size_t i = 0; i < names.size(); ++i) {
      std::cout << names[i] << std::endl;
    }
  }
}
std::string ListTasks::name() const { return "list"; }
std::string ListTasks::description() const { return "- List tasks"; }

DeleteTask::DeleteTask(rapid::db::NameDb* db) : db_(db) {}
void DeleteTask::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No task to delete.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  bool success = db_->Delete<object_search_msgs::Task>(name);
  if (!success) {
    ROS_ERROR("Task \"%s\" not found.", name.c_str());
    return;
  }
}
std::string DeleteTask::name() const { return "delete"; }
std::string DeleteTask::description() const { return "<name> - Delete a task"; }

ShowTask::ShowTask(const ExperimentDbs& dbs, const ExperimentVizs& vizs)
    : dbs_(dbs), vizs_(vizs) {}
void ShowTask::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No task to show.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  object_search_msgs::Task task;
  bool success = dbs_.task_db->Get(name, &task);
  if (!success) {
    ROS_ERROR("Task \"%s\" not found.", name.c_str());
    return;
  }

  vizs_.scene_viz->Clear();
  sensor_msgs::PointCloud2 scene_cloud;
  success = dbs_.scene_cloud_db->Get(task.scene_name, &scene_cloud);
  if (!success) {
    ROS_WARN("Scene cloud \"%s\" not found.", task.scene_name.c_str());
  } else {
    vizs_.scene_viz->set_scene(scene_cloud);
  }

  // TODO: visualize labels
}
std::string ShowTask::name() const { return "show"; }
std::string ShowTask::description() const {
  return "<task> - Visualize a task";
}

CreateTask::CreateTask(const ExperimentDbs& dbs) : dbs_(dbs) {}
void CreateTask::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No name specified.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  object_search_msgs::Task task;
  task.name = name;
  dbs_.task_db->Insert(name, task);

  std::cout << "Created task named \"" << name << "\"." << std::endl;
}
std::string CreateTask::name() const { return "create"; }
std::string CreateTask::description() const { return "<name> - Create a task"; }

EditTask::EditTask(const ExperimentDbs& dbs) : dbs_(dbs) {}
void EditTask::Execute(const std::vector<std::string>& args) {
  if (args.size() <= 0) {
    ROS_ERROR("No name specified.");
    return;
  }

  std::string name(boost::algorithm::join(args, " "));
  object_search_msgs::Task task;
  dbs_.task_db->Get(name, &task);
}
std::string EditTask::name() const { return "edit"; }
std::string EditTask::description() const { return "<name> - Edit a task"; }
}  // namespace object_search
