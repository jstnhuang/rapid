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
}  // namespace object_search
