#ifndef _RAPID_DISPLAY_DISPLAY_H_
#define _RAPID_DISPLAY_DISPLAY_H_

#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "gmock/gmock.h"

namespace rapid {
namespace display {

// Display methods return true/false for success.
class DisplayInterface {
 public:
  virtual ~DisplayInterface(){};
  virtual bool ShowDefault() = 0;
  virtual bool ShowMessage(const std::string& h1_text,
                           const std::string& h2_text) = 0;
  virtual bool AskMultipleChoice(const std::string& question,
                                 const std::vector<std::string>& choices,
                                 std::string* choice) = 0;
};

class Blinky : public DisplayInterface {
 public:
  Blinky(actionlib::SimpleActionClient<blinky::FaceAction>& client);
  virtual bool ShowDefault();
  virtual bool ShowMessage(const std::string& h1_text,
                           const std::string& h2_text);

  // Asks the given question and stores the result in choice.
  virtual bool AskMultipleChoice(const std::string& question,
                                 const std::vector<std::string>& choices,
                                 std::string* choice);

 private:
  bool WaitForServer(const int seconds);
  static const int kServerWaitTime = 5;  // In seconds.
  actionlib::SimpleActionClient<blinky::FaceAction>& client_;
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
