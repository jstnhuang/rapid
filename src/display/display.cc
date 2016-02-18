#include "rapid/display/display.h"

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "ros/ros.h"

#include <iostream>

namespace rapid {
namespace display {
Blinky::Blinky() : client_("blinky"), server_wait_time_(5) {}
Blinky::Blinky(int server_wait_time)
    : client_("blinky"), server_wait_time_(server_wait_time) {}

bool Blinky::ShowDefault() {
  if (!WaitForServer(server_wait_time_)) {
    ROS_ERROR("Timed out waiting for the Blinky server.");
    return false;
  }
  blinky::FaceGoal goal;
  goal.display_type = goal.DEFAULT;
  client_.sendGoal(goal);
  std::cerr << "Waiting for result" << std::endl;
  if (!client_.waitForResult(ros::Duration(server_wait_time_))) {
    ROS_ERROR("Timed out waiting for Blinky result.");
    std::cerr << "Timed out waiting for result" << std::endl;
    return false;
  }
  std::cerr << "Done waiting for result" << std::endl;
  return true;
}

bool Blinky::ShowMessage(const std::string& h1_text,
                         const std::string& h2_text) {
  if (!WaitForServer(server_wait_time_)) {
    ROS_ERROR("Timed out waiting for the Blinky server.");
    return false;
  }
  blinky::FaceGoal goal;
  goal.display_type = goal.DISPLAY_MESSAGE;
  goal.h1_text = h1_text;
  goal.h2_text = h2_text;
  client_.sendGoal(goal);
  std::cerr << "Waiting for result" << std::endl;
  if (!client_.waitForResult(ros::Duration(server_wait_time_))) {
    ROS_ERROR("Timed out waiting for Blinky result.");
    std::cerr << "Timed out waiting for result" << std::endl;
    return false;
  }
  std::cerr << "Done waiting for result" << std::endl;
  return true;
}
bool Blinky::AskMultipleChoice(const std::string& question,
                               const std::vector<std::string>& choices,
                               std::string* choice) {
  if (!WaitForServer(server_wait_time_)) {
    ROS_ERROR("Timed out waiting for the Blinky server.");
    return false;
  }
  blinky::FaceGoal goal;
  goal.display_type = goal.ASK_CHOICE;
  goal.question = question;
  goal.choices = choices;
  client_.sendGoal(goal);
  std::cerr << "Waiting for result" << std::endl;
  if (!client_.waitForResult()) {
    ROS_ERROR("Timed out waiting for Blinky result.");
    std::cerr << "Timed out waiting for result" << std::endl;
    return false;
  }
  std::cerr << "Done waiting for result" << std::endl;
  blinky::FaceResultConstPtr result = client_.getResult();
  *choice = result->choice;
  return true;
}

bool Blinky::WaitForServer(const int seconds) {
  ros::Duration duration(seconds);
  return client_.waitForServer(duration);
}
}  // namespace display
}  // namespace rapid
