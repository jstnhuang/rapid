#ifndef _RAPID_DISPLAY_DISPLAY_H_
#define _RAPID_DISPLAY_DISPLAY_H_

#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "gmock/gmock.h"

#include "rapid_ros/action_client.h"

namespace rapid {
namespace display {
// Interface for UI display on a screen.
// Display methods return true/false for success.
class DisplayInterface {
 public:
  virtual ~DisplayInterface(){};

  // Show the "default" screen.
  virtual bool ShowDefault() = 0;

  // Show a message, consisting of large (h1) text and smaller (h2) text.
  virtual bool ShowMessage(const std::string& h1_text,
                           const std::string& h2_text) = 0;

  // Ask a multiple choice question.
  virtual bool AskMultipleChoice(const std::string& question,
                                 const std::vector<std::string>& choices,
                                 std::string* choice) = 0;
};

class Blinky : public DisplayInterface {
 public:
  // Constructs Blinky with the given action client.
  // Blinky takes ownership of the pointer and will delete it when destructed.
  explicit Blinky(rapid_ros::ActionClientInterface<blinky::FaceAction>* client);
  ~Blinky();
  virtual bool ShowDefault();
  virtual bool ShowMessage(const std::string& h1_text,
                           const std::string& h2_text);

  // Asks the given question and stores the result in choice.
  virtual bool AskMultipleChoice(const std::string& question,
                                 const std::vector<std::string>& choices,
                                 std::string* choice);
  void set_server_wait_time(int seconds) { server_wait_time_ = seconds; }
  int server_wait_time() { return server_wait_time_; }

 private:
  bool WaitForServer(const int seconds);

  rapid_ros::ActionClientInterface<blinky::FaceAction>* client_;
  int server_wait_time_;  // In seconds.
};

class MockDisplay : public DisplayInterface {
 public:
  MockDisplay() {}
  MOCK_METHOD0(ShowDefault, bool());
  MOCK_METHOD2(ShowMessage,
               bool(const std::string& h1_text, const std::string& h2_text));
  MOCK_METHOD3(AskMultipleChoice, bool(const std::string& question,
                                       const std::vector<std::string>& choices,
                                       std::string* choice));
};
}  // namespace display
}  // namespace rapid
#endif  // _RAPID_DISPLAY_DISPLAY_H_
