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
  virtual void Execute(const std::vector<std::string>& args) = 0;

  // The name of the command (e.g., what the user types to run it.
  virtual std::string name() const = 0;

  // A description of the command.
  virtual std::string description() const = 0;
};
}  // namespace utils
}  // namespace rapid

#endif  // _RAPID_UTILS_COMMAND_H_
