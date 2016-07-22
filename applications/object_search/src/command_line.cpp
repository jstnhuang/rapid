#include "object_search/command_line.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "boost/algorithm/string.hpp"

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;

namespace object_search {
CommandLine::CommandLine() {}

bool CommandLine::Next() {
  string command;
  vector<string> args;
  GetCommand(&command, &args);
  if (command == "objects") {
    list_objects_->Execute(args);
  } else if (command == "scenes") {
    list_scenes_->Execute(args);
  } else if (command == "record object") {
    record_object_->Execute(args);
  } else if (command == "record scene") {
    record_scene_->Execute(args);
  } else if (command == "delete object") {
    delete_object_->Execute(args);
  } else if (command == "delete scene") {
    delete_scene_->Execute(args);
  } else if (command == "use object") {
    use_object_->Execute(args);
  } else if (command == "use scene") {
    use_scene_->Execute(args);
  } else if (command == "run") {
    run_->Execute(args);
  } else if (command == "debug") {
    debug_->Execute(args);
  } else if (command == "exit") {
    return false;
  }
  cout << endl;
  return true;
}

void CommandLine::set_list_objects(Command* list_objects) {
  list_objects_ = list_objects;
}
void CommandLine::set_list_scenes(Command* list_scenes) {
  list_scenes_ = list_scenes;
}
void CommandLine::set_record_object(Command* record_object) {
  record_object_ = record_object;
}
void CommandLine::set_record_scene(Command* record_scene) {
  record_scene_ = record_scene;
}
void CommandLine::set_delete_object(Command* delete_object) {
  delete_object_ = delete_object;
}
void CommandLine::set_delete_scene(Command* delete_scene) {
  delete_scene_ = delete_scene;
}
void CommandLine::set_use_object(Command* use_object) {
  use_object_ = use_object;
}
void CommandLine::set_use_scene(Command* use_scene) { use_scene_ = use_scene; }
void CommandLine::set_run(Command* run) { run_ = run; }
void CommandLine::set_debug(Command* debug) { debug_ = debug; }

void CommandLine::ShowCommands() {
  cout << "Commands:" << endl;
  cout << "  objects - List objects" << endl;
  cout << "  scenes - List scenes" << endl;
  cout << "  record object <name> - Save a new object" << endl;
  cout << "  record scene <name> - Save a new scene" << endl;
  cout << "  delete object <name> - Delete an object" << endl;
  cout << "  delete scene <name> - Delete a scene" << endl;
  cout << "  use object <name> - Set object to search for" << endl;
  cout << "  use scene <name> - Set the scene to search in" << endl;
  cout << "  run - Run object search" << endl;
  cout << "  debug <on/off> - Turn debugging on or off" << endl;
  cout << "  exit - Exit this application" << endl;
}

bool CommandLine::ParseCommand(const string& input, string* command,
                               vector<string>* args) {
  vector<string> tokens;
  boost::split(tokens, input, boost::is_space());
  if (tokens.size() == 0) {
    return false;
    ;
  }
  *command = tokens[0];
  args->clear();
  if (*command == "objects") {
    return true;
  } else if (*command == "scenes") {
    return true;
  } else if (*command == "record") {
    if (tokens.size() < 3) {
      return false;
    }
    if (tokens[1] == "object") {
      *command = "record object";
    } else if (tokens[1] == "scene") {
      *command = "record scene";
    } else {
      return false;
    }
    string name;
    ParseName(tokens, 2, &name);
    args->push_back(name);
    return true;
  } else if (*command == "delete") {
    if (tokens.size() < 3) {
      return false;
    }
    if (tokens[1] == "object") {
      *command = "delete object";
    } else if (tokens[1] == "scene") {
      *command = "delete scene";
    } else {
      return false;
    }
    string name;
    ParseName(tokens, 2, &name);
    args->push_back(name);
    return true;
  } else if (*command == "use") {
    if (tokens.size() < 3) {
      return false;
    }
    if (tokens[1] == "object") {
      *command = "use object";
    } else if (tokens[1] == "scene") {
      *command = "use scene";
    } else {
      return false;
    }
    string name;
    ParseName(tokens, 2, &name);
    args->push_back(name);
    return true;
  } else if (*command == "run") {
    return true;
  } else if (*command == "debug") {
    if (tokens.size() < 2) {
      return false;
    }
    if (tokens[1] == "on" || tokens[1] == "off") {
      args->push_back(tokens[1]);
      return true;
    } else {
      cout << tokens[1] << " must be either \"on\" or \"off\"" << endl;
      return false;
    }
  } else if (*command == "exit") {
    return true;
  }
  return false;
}

void CommandLine::GetCommand(string* command, vector<string>* args) {
  string input;
  bool valid = false;
  while (!valid) {
    ShowCommands();
    cout << "Enter a command: ";
    if (std::getline(cin, input)) {
      cout << endl;
      valid = ParseCommand(input, command, args);
      if (!valid) {
        cout << "Invalid command. " << endl;
      }
    } else {
      cout << endl;
      *command = "exit";
      valid = true;
    }
  }
}

void CommandLine::ParseName(const std::vector<std::string>& tokens, int start,
                            std::string* name) {
  std::stringstream ss;
  for (size_t i = start; i < tokens.size(); ++i) {
    ss << tokens[i];
    if (i != tokens.size() - 1) {
      ss << " ";
    }
  }
  *name = ss.str();
}
}  // namespace object_search
