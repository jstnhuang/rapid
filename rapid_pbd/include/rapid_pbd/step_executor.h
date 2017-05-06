#ifndef _RAPID_PBD_STEP_EXECUTOR_H_
#define _RAPID_PBD_STEP_EXECUTOR_H_

#include "boost/shared_ptr.hpp"

#include "rapid_pbd/action_executor.h"
#include "rapid_pbd_msgs/Step.h"

namespace rapid {
namespace pbd {
class StepExecutor {
 public:
  explicit StepExecutor(const rapid_pbd_msgs::Step& step);

  // Returns true if the Step message is valid, false otherwise.
  // You should call this method to verify the step message before executing it.
  bool IsValid() const;

  // Starts the step execution.
  void Start();

  // Returns true if the step execution is complete or if the step is invalid.
  bool IsDone() const;

  // Cancels the execution of the step.
  void Cancel();

 private:
  rapid_pbd_msgs::Step step_;
  std::vector<boost::shared_ptr<ActionExecutor> > executors_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_STEP_EXECUTOR_H_
