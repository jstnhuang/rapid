#ifndef _OBJECT_SEARCH_COMMAND_LINE_H_
#define _OBJECT_SEARCH_COMMAND_LINE_H_

#include <string>
#include <vector>

#include "object_search/commands.h"

namespace object_search {
class CommandLine {
 public:
  CommandLine();
  bool Next();  // Returns false on exit.

  void set_list_objects(Command* list_objects);
  void set_list_scenes(Command* list_scenes);
  void set_record_object(Command* record_object);
  void set_record_scene(Command* record_scene);
  void set_delete_object(Command* delete_object);
  void set_delete_scene(Command* delete_scene);
  void set_use_object(Command* use_object);
  void set_use_scene(Command* use_scene);
  void set_run(Command* run);
  void set_debug(Command* debug);

 private:
  void ShowCommands();
  bool ParseCommand(const std::string& input, std::string* command,
                    std::vector<std::string>* args);
  void GetCommand(std::string* command, std::vector<std::string>* args);
  void ParseName(const std::vector<std::string>& tokens, int start,
                 std::string* name);

  Command* list_objects_;
  Command* list_scenes_;
  Command* record_object_;
  Command* record_scene_;
  Command* delete_object_;
  Command* delete_scene_;
  Command* use_object_;
  Command* use_scene_;
  Command* run_;
  Command* debug_;
};
}  // namespace object_search

#endif  // _OBJECT_SEARCH_COMMAND_LINE_H_
