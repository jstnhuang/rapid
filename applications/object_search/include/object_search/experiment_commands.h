#ifndef _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_
#define _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_

#include "rapid_db/name_db.hpp"
#include "rapid_utils/command_interface.h"

namespace object_search {
class ListTasks : public rapid::utils::CommandInterface {
 public:
  ListTasks(rapid::db::NameDb* db);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* db_;
};

class DeleteTask : public rapid::utils::CommandInterface {
 public:
  DeleteTask(rapid::db::NameDb* db);
  void Execute(const std::vector<std::string>& args);
  std::string name() const;
  std::string description() const;

 private:
  rapid::db::NameDb* db_;
};

}  // namespace object_search

#endif  // _OBJECT_SEARCH_EXPERIMENT_COMMANDS_H_
