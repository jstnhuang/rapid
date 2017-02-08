#include "rapid_utils/command_line.h"

#include <iostream>

#include "boost/algorithm/string.hpp"
#include "readline/history.h"
#include "readline/readline.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;

namespace rapid {
namespace utils {
void ExitCommand::Execute(const std::vector<std::string>& args) {}
std::string ExitCommand::name() const { return "exit"; }
std::string ExitCommand::description() const { return "- Exit this interface"; }

CommandLine::CommandLine() : name_(""), commands_() {}
CommandLine::CommandLine(const std::string& name) : name_(name), commands_() {}

void CommandLine::AddCommand(CommandInterface* command) {
  commands_.push_back(command);
}

bool CommandLine::Next() {
  ShowCommands();
  char* line = readline("Enter a command: ");
  add_history(line);
  if (line) {
    cout << endl;
  } else {
    cout << endl;
    return false;
  }

  CommandInterface* command;
  vector<string> args;
  bool valid = ParseLine(std::string(line), &command, &args);
  if (valid) {
    if (command->name() == "exit") {
      delete line;
      return false;
    }
    command->Execute(args);
    cout << endl;
  } else {
    cout << "Invalid command." << endl;
    cout << endl;
  }
  delete line;
  return true;
}

void CommandLine::ShowCommands() const {
  if (name_ != "") {
    cout << name_ << " - ";
  }
  cout << "Commands:" << endl;
  for (size_t i = 0; i < commands_.size(); ++i) {
    CommandInterface* command = commands_[i];
    cout << "  " << command->name() << " " << command->description() << endl;
  }
}

bool CommandLine::ParseLine(const std::string& line,
                            CommandInterface** command_pp,
                            std::vector<std::string>* args) const {
  vector<string> tokens;
  boost::split(tokens, line, boost::is_space());
  for (size_t i = 0; i < commands_.size(); ++i) {
    CommandInterface* command = commands_[i];
    int match_length = ParseCommand(tokens, command->name());
    if (match_length == 0) {
      continue;
    }

    // At this point, we have a matching command, which we return.
    *command_pp = command;
    args->clear();
    for (size_t j = match_length; j < tokens.size(); ++j) {
      args->push_back(tokens[j]);
    }
    return true;
  }
  return false;
}

int CommandLine::ParseCommand(const std::vector<std::string>& tokens,
                              const std::string& name) const {
  vector<string> command_tokens;
  boost::split(command_tokens, name, boost::is_space());
  if (command_tokens.size() > tokens.size()) {
    return 0;
  }
  for (size_t j = 0; j < command_tokens.size(); ++j) {
    if (command_tokens[j] != tokens[j]) {
      return 0;
    }
  }
  return command_tokens.size();
}
}  // namespace utils
}  // namespace rapid
