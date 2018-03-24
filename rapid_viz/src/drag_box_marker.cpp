#include "rapid_viz/drag_box_marker.h"

#include "boost/bind.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "interactive_markers/interactive_marker_server.h"
#include "tf/transform_datatypes.h"
#include "transform_graph/graph.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/Marker.h"

using geometry_msgs::PoseStamped;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::Marker;

namespace rapid {
DragBoxMarker::DragBoxMarker(
    const std::string& name,
    interactive_markers::InteractiveMarkerServer* im_server)
    : name_(name),
      im_server_(im_server),
      pose_stamped_(),
      min_x_(-0.05),
      max_x_(0.05),
      min_y_(-0.05),
      max_y_(0.05),
      min_z_(-0.05),
      max_z_(0.05),
      tf_graph_() {}

void DragBoxMarker::Show() {
  Update();
  im_server_->setCallback(name_ + "-pos_x",
                          boost::bind(&DragBoxMarker::Feedback, this, _1));
  im_server_->setCallback(name_ + "-neg_x",
                          boost::bind(&DragBoxMarker::Feedback, this, _1));
  im_server_->setCallback(name_ + "-pos_y",
                          boost::bind(&DragBoxMarker::Feedback, this, _1));
  im_server_->setCallback(name_ + "-neg_y",
                          boost::bind(&DragBoxMarker::Feedback, this, _1));
  im_server_->setCallback(name_ + "-pos_z",
                          boost::bind(&DragBoxMarker::Feedback, this, _1));
  im_server_->setCallback(name_ + "-neg_z",
                          boost::bind(&DragBoxMarker::Feedback, this, _1));
  im_server_->applyChanges();
}

void DragBoxMarker::Hide() {
  im_server_->erase(name_ + "-box");
  im_server_->erase(name_ + "-pos_x");
  im_server_->erase(name_ + "-pos_y");
  im_server_->erase(name_ + "-pos_z");
  im_server_->erase(name_ + "-neg_x");
  im_server_->erase(name_ + "-neg_y");
  im_server_->erase(name_ + "-neg_z");
  im_server_->applyChanges();
}

void DragBoxMarker::set_pose_stamped(const PoseStamped& pose_stamped) {
  pose_stamped_ = pose_stamped;
  tf_graph_.Add("box", transform_graph::RefFrame(pose_stamped_.header.frame_id),
                pose_stamped.pose);
  Update();
}

void DragBoxMarker::set_min_x(double min_x) {
  min_x_ = min_x;
  Update();
}
void DragBoxMarker::set_max_x(double max_x) {
  max_x_ = max_x;
  Update();
}
void DragBoxMarker::set_min_y(double min_y) {
  min_y_ = min_y;
  Update();
}
void DragBoxMarker::set_max_y(double max_y) {
  max_y_ = max_y;
  Update();
}
void DragBoxMarker::set_min_z(double min_z) {
  min_z_ = min_z;
  Update();
}
void DragBoxMarker::set_max_z(double max_z) {
  max_z_ = max_z;
  Update();
}

geometry_msgs::PoseStamped DragBoxMarker::pose_stamped() {
  return pose_stamped_;
}
double DragBoxMarker::min_x() { return min_x_; }
double DragBoxMarker::min_y() { return min_y_; }
double DragBoxMarker::min_z() { return min_z_; }
double DragBoxMarker::max_x() { return max_x_; }
double DragBoxMarker::max_y() { return max_y_; }
double DragBoxMarker::max_z() { return max_z_; }

void DragBoxMarker::Update() {
  InteractiveMarker box;
  MakeBox(&box);
  im_server_->insert(box);

  InteractiveMarker x_arrow;
  MakeArrow("x", "pos", &x_arrow);
  im_server_->insert(x_arrow);

  InteractiveMarker y_arrow;
  MakeArrow("y", "pos", &y_arrow);
  im_server_->insert(y_arrow);

  InteractiveMarker z_arrow;
  MakeArrow("z", "pos", &z_arrow);
  im_server_->insert(z_arrow);

  InteractiveMarker neg_x_arrow;
  MakeArrow("x", "neg", &neg_x_arrow);
  im_server_->insert(neg_x_arrow);

  InteractiveMarker neg_y_arrow;
  MakeArrow("y", "neg", &neg_y_arrow);
  im_server_->insert(neg_y_arrow);

  InteractiveMarker neg_z_arrow;
  MakeArrow("z", "neg", &neg_z_arrow);
  im_server_->insert(neg_z_arrow);

  im_server_->applyChanges();
}

void DragBoxMarker::UpdateArrow(const std::string& axis,
                                const std::string& polarity,
                                const geometry_msgs::Point& point) {
  if (polarity == "pos") {
    if (axis == "x") {
      max_x_ = point.x - 0.05;
      min_x_ = std::min(min_x_, max_x_ - 0.01);
    } else if (axis == "y") {
      max_y_ = point.y - 0.05;
      min_y_ = std::min(min_y_, max_y_ - 0.01);
    } else if (axis == "z") {
      max_z_ = point.z - 0.05;
      min_z_ = std::min(min_z_, max_z_ - 0.01);
    }
  } else if (polarity == "neg") {
    if (axis == "x") {
      min_x_ = point.x + 0.05;
      max_x_ = std::max(max_x_, min_x_ + 0.01);
    } else if (axis == "y") {
      min_y_ = point.y + 0.05;
      max_y_ = std::max(max_y_, min_y_ + 0.01);
    } else if (axis == "z") {
      min_z_ = point.z + 0.05;
      max_z_ = std::max(max_z_, min_z_ + 0.01);
    }
  }
  Update();
}

void DragBoxMarker::Feedback(
    const InteractiveMarkerFeedback::ConstPtr& feedback) {
  if (feedback->event_type != InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }

  transform_graph::Position local_position;
  tf_graph_.DescribePosition(feedback->pose.position,
                             transform_graph::Source(feedback->header.frame_id),
                             transform_graph::Target("box"), &local_position);
  geometry_msgs::Point local_point;
  local_point.x = local_position.vector().x();
  local_point.y = local_position.vector().y();
  local_point.z = local_position.vector().z();
  if (feedback->marker_name == name_ + "-pos_x") {
    UpdateArrow("x", "pos", local_point);
  } else if (feedback->marker_name == name_ + "-pos_y") {
    UpdateArrow("y", "pos", local_point);
  } else if (feedback->marker_name == name_ + "-pos_z") {
    UpdateArrow("z", "pos", local_point);
  } else if (feedback->marker_name == name_ + "-neg_x") {
    UpdateArrow("x", "neg", local_point);
  } else if (feedback->marker_name == name_ + "-neg_y") {
    UpdateArrow("y", "neg", local_point);
  } else if (feedback->marker_name == name_ + "-neg_z") {
    UpdateArrow("z", "neg", local_point);
  }
}

void DragBoxMarker::MakeBox(InteractiveMarker* box) {
  double x = (min_x_ + max_x_) / 2;
  double y = (min_y_ + max_y_) / 2;
  double z = (min_z_ + max_z_) / 2;
  double dim_x = max_x_ - min_x_;
  double dim_y = max_y_ - min_y_;
  double dim_z = max_z_ - min_z_;
  Marker marker;
  marker.type = Marker::CUBE;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale.x = dim_x;
  marker.scale.y = dim_y;
  marker.scale.z = dim_z;
  marker.color.b = 1;
  marker.color.a = 0.3;

  InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = InteractiveMarkerControl::NONE;
  control.markers.push_back(marker);

  box->name = name_ + "-box";
  box->header = pose_stamped_.header;
  box->pose = pose_stamped_.pose;
  box->scale = 1;
  box->controls.clear();
  box->controls.push_back(control);
}

void DragBoxMarker::MakeArrow(const std::string& axis,
                              const std::string& polarity,
                              visualization_msgs::InteractiveMarker* arrow) {
  Marker marker;
  marker.type = Marker::ARROW;
  marker.scale.x = 0.05;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  if (axis == "x") {
    if (polarity == "pos") {
      marker.color.r = 1;
    } else {
      marker.color.r = 0.5;
    }
  } else if (axis == "y") {
    if (polarity == "pos") {
      marker.color.g = 1;
    } else {
      marker.color.g = 0.5;
    }
  } else if (axis == "z") {
    if (polarity == "pos") {
      marker.color.b = 1;
    } else {
      marker.color.b = 0.5;
    }
  }
  marker.color.a = 0.5;

  tf::Quaternion arrow_q;
  geometry_msgs::Pose arrow_pose;
  arrow_pose.position.x = (max_x_ + min_x_) / 2;
  arrow_pose.position.y = (max_y_ + min_y_) / 2;
  arrow_pose.position.z = (max_z_ + min_z_) / 2;
  if (polarity == "pos") {
    if (axis == "x") {
      arrow_q = tf::Quaternion(0, 0, 0, 1);
      arrow_pose.position.x = max_x_ + 0.05;
    } else if (axis == "y") {
      arrow_q = tf::Quaternion(0, 0, 1, 1);
      arrow_pose.position.y = max_y_ + 0.05;
    } else if (axis == "z") {
      arrow_q = tf::Quaternion(0, -1, 0, 1);
      arrow_pose.position.z = max_z_ + 0.05;
    }
  } else if (polarity == "neg") {
    if (axis == "x") {
      arrow_q = tf::Quaternion(0, 0, 1, 0);
      arrow_pose.position.x = min_x_ - 0.05;
    } else if (axis == "y") {
      arrow_q = tf::Quaternion(0, 0, -1, 1);
      arrow_pose.position.y = min_y_ - 0.05;
    } else if (axis == "z") {
      arrow_q = tf::Quaternion(0, 1, 0, 1);
      arrow_pose.position.z = min_z_ - 0.05;
    }
  }
  arrow_q.normalize();
  tf::quaternionTFToMsg(arrow_q, marker.pose.orientation);

  InteractiveMarkerControl control;
  control.orientation.w = 1;

  double sign = 1;
  if (polarity == "neg") {
    sign = -1;
  }
  if (axis == "x") {
    if (sign == -1) {
      control.orientation.w = 0;
      control.orientation.z = 1;
    }
  } else if (axis == "y") {
    control.orientation.z = sign;
  } else if (axis == "z") {
    control.orientation.y = -sign;
  }
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  control.always_visible = true;
  control.markers.push_back(marker);

  transform_graph::Transform global_arrow_pose_tf;
  tf_graph_.DescribePose(arrow_pose, transform_graph::Source("box"),
                         transform_graph::Target(pose_stamped_.header.frame_id),
                         &global_arrow_pose_tf);

  arrow->name = name_ + "-" + polarity + "_" + axis;
  arrow->header.frame_id = pose_stamped_.header.frame_id;
  global_arrow_pose_tf.ToPose(&arrow->pose);
  arrow->controls.push_back(control);
  arrow->scale = 0.05;
}
}  // namespace rapid
