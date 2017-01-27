#ifndef _RAPID_UTILS_COMMAND_H_
#define _RAPID_UTILS_COMMAND_H_

#include <string>
#include <vector>

namespace rapid {
namespace utils {
class CommandInterface {
 public:
  virtual ~CommandInterface() {}

  // Execute the command.
  void Execute(const std::vector<std::string>& args);

  // Return true if the user input is valid for this command, false otherwise.
  bool Parse(const std::vector<std::string>& input) const;

  // The name of the command (e.g., what the user types to run it.
  std::string name() const;

  // A description of the command.
  std::string description() const;
};
}  // namespace utils
}  // namespace rapid

#endif  // _RAPID_UTILS_COMMAND_H_
