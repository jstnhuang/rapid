#include "rapid_display/display.h"

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "ros/ros.h"

#include "rapid_ros/action_client.h"

using ros::Duration;

namespace rapid {
namespace display {
Blinky::Blinky(rapid_ros::ActionClientInterface<blinky::FaceAction>* client)
    : client_(client), server_wait_time_(5) {}

Blinky::~Blinky() {
  if (client_ != NULL) {
    delete client_;
  } else {
    ROS_ERROR("Pointer to ActionClient was null!");
  }
}

bool Blinky::ShowDefault() {
  if (client_ == NULL) {
    ROS_ERROR("Pointer to ActionClient was null!");
    return false;
  }
  if (!WaitForServer(server_wait_time_)) {
    ROS_ERROR("Timed out waiting for the Blinky server.");
    return false;
  }
  blinky::FaceGoal goal;
  goal.display_type = goal.DEFAULT;
  client_->sendGoal(goal);
  if (!client_->waitForResult(Duration(server_wait_time_))) {
    ROS_ERROR("Timed out waiting for Blinky result.");
    return false;
  }
  return true;
}

bool Blinky::ShowMessage(const std::string& h1_text,
                         const std::string& h2_text) {
  if (client_ == NULL) {
    ROS_ERROR("Pointer to ActionClient was null!");
    return false;
  }
  if (!WaitForServer(server_wait_time_)) {
    ROS_ERROR("Timed out waiting for the Blinky server.");
    return false;
  }
  blinky::FaceGoal goal;
  goal.display_type = goal.DISPLAY_MESSAGE;
  goal.h1_text = h1_text;
  goal.h2_text = h2_text;
  client_->sendGoal(goal);
  if (!client_->waitForResult(Duration(server_wait_time_))) {
    ROS_ERROR("Timed out waiting for Blinky result.");
    return false;
  }
  return true;
}

bool Blinky::AskMultipleChoice(const std::string& question,
                               const std::vector<std::string>& choices,
                               std::string* choice) {
  if (client_ == NULL) {
    ROS_ERROR("Pointer to ActionClient was null!");
    return false;
  }
  if (!WaitForServer(server_wait_time_)) {
    ROS_ERROR("Timed out waiting for the Blinky server.");
    return false;
  }
  blinky::FaceGoal goal;
  goal.display_type = goal.ASK_CHOICE;
  goal.question = question;
  goal.choices = choices;
  client_->sendGoal(goal);
  client_->waitForResult();
  blinky::FaceResultConstPtr result = client_->getResult();
  *choice = result->choice;
  return true;
}

bool Blinky::WaitForServer(const int seconds) {
  if (client_ == NULL) {
    ROS_ERROR("Pointer to ActionClient was null!");
    return false;
  }
  Duration duration(seconds);
  return client_->waitForServer(duration);
}
}  // namespace display
}  // namespace rapid
