#ifndef _RAPID_ROS_PUBLISHER_H_
#define _RAPID_ROS_PUBLISHER_H_

#include <vector>

#include "ros/ros.h"

namespace rapid_ros {
// DECLARATIONS ----------------------------------------------------------------
// Interface wrapper for ros::Publisher
template <class Message>
class PublisherInterface {
 public:
  virtual ~PublisherInterface() {}
  virtual void publish(const Message& message) const = 0;
  virtual bool IsValid() const = 0;
};

// A wrapper around a real ROS publisher.
template <class Message>
class Publisher : public PublisherInterface<Message> {
 public:
  explicit Publisher(const ros::Publisher& pub);
  void publish(const Message& message) const;
  bool IsValid() const;

 private:
  ros::Publisher pub_;
};

template <class Message>
struct MessageHistory {
  std::vector<Message> messages;
};

// A mock publisher.
template <class Message>
class MockPublisher : public PublisherInterface<Message> {
 public:
  MockPublisher();
  void publish(const Message& message) const;
  bool IsValid() const;

  // Mock helpers
  Message last_message() const;
  std::vector<Message> sent_messages() const;
  void set_valid(bool valid);

 private:
  MessageHistory<Message>* sent_;
  bool is_valid_;
};

// DEFINITIONS -----------------------------------------------------------------
template <class Message>
Publisher<Message>::Publisher(const ros::Publisher& pub)
    : pub_(pub) {}

template <class Message>
void Publisher<Message>::publish(const Message& message) const {
  pub_.publish(message);
}

template <class Message>
bool Publisher<Message>::IsValid() const {
  if (pub_) {
    return true;
  } else {
    return false;
  }
}

template <class Message>
MockPublisher<Message>::MockPublisher()
    : sent_(new MessageHistory<Message>), is_valid_(true) {}

template <class Message>
void MockPublisher<Message>::publish(const Message& message) const {
  if (is_valid_) {
    sent_->messages.push_back(message);
  }
}

template <class Message>
bool MockPublisher<Message>::IsValid() const {
  return is_valid_;
}

template <class Message>
Message MockPublisher<Message>::last_message() const {
  return sent_->messages.back();
}

template <class Message>
std::vector<Message> MockPublisher<Message>::sent_messages() const {
  return sent_->messages;
}

template <class Message>
void MockPublisher<Message>::set_valid(bool valid) {
  is_valid_ = valid;
}
}  // namespace rapid_ros

#endif  // _RAPID_ROS_PUBLISHER_H_
