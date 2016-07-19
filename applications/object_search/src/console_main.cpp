#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "boost/algorithm/string.hpp"
#include "ros/ros.h"

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::vector;

void ShowCommands();
bool ParseCommand(const string& input, string* command, vector<string>* args);
void GetCommand(string* command, vector<string>* args);

class Client {
 public:
  Client();
  void Create(const string& name);
  void List();
  void Delete(const string& id);
};

Client::Client() {}

void Client::Create(const string& name) {}
void Client::List() {}
void Client::Delete(const string& id) {}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_search_console");
  Client client;
  while (true) {
    string command;
    vector<string> args;
    GetCommand(&command, &args);
    if (command == "exit") {
      break;
    }
    if (command == "create") {
      client.Create(args[0]);
    } else if (command == "list") {
      client.List();
    } else if (command == "delete") {
      client.Delete(args[0]);
    }
  }
  return 0;
}

void ShowCommands() {
  cout << "Commands:" << endl;
  cout << "  objects - List objects" << endl;
  cout << "  scenes - List scenes" << endl;
  cout << "  record object <name> - Save a new object" << endl;
  cout << "  record scene <name> - Save a new scene" << endl;
  cout << "  delete object <id> - Delete an object" << endl;
  cout << "  delete scene <id> - Delete a scene" << endl;
  cout << "  use object <id> - Set object to search for" << endl;
  cout << "  use scene <id> - Set the scene to search in" << endl;
  cout << "  run - Run object search" << endl;
  cout << "  debug <on/off> - Turn debugging on or off" << endl;
  cout << "  exit - Exit this application" << endl;
}

bool ParseCommand(const string& input, string* command, vector<string>* args) {
  vector<string> tokens;
  boost::split(tokens, input, boost::is_space());
  if (tokens.size() == 0) {
    return false;
    ;
  }
  *command = tokens[0];
  args->clear();
  if (*command == "create") {
    if (tokens.size() == 1) {
      return false;
    }
    std::stringstream name;
    for (size_t i = 1; i < tokens.size(); ++i) {
      name << tokens[i];
      if (i != tokens.size() - 1) {
        name << " ";
      }
    }
    args->push_back(name.str());
    return true;
  } else if (*command == "list") {
    return true;
  } else if (*command == "delete") {
    if (tokens.size() == 1) {
      return false;
    }
    args->push_back(tokens[1]);
    return true;
  } else if (*command == "exit") {
    return true;
  }
  return false;
}

void GetCommand(string* command, vector<string>* args) {
  string input;
  bool valid = false;
  while (!valid) {
    ShowCommands();
    cout << "Enter a command: ";
    cin >> input;
    valid = ParseCommand(input, command, args);
  }
}
