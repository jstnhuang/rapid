#include "rapid_perception/box3d_roi_server.h"

#include <math.h>
#include <string>

#include "boost/bind.hpp"
#include "interactive_markers/interactive_marker_server.h"
#include "rapid_msgs/Roi3D.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include "visualization_msgs/Marker.h"

using interactive_markers::InteractiveMarkerServer;
using rapid_msgs::Roi3D;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using visualization_msgs::Marker;

namespace rapid {
namespace perception {
Box3DRoiServer::Box3DRoiServer(const std::string& topic)
    : server_(new InteractiveMarkerServer(topic)),
      base_frame_("base_footprint") {}

Box3DRoiServer::~Box3DRoiServer() {
  if (server_ != NULL) {
    delete server_;
  }
}

InteractiveMarker Box3DRoiServer::Box(double x, double y, double z,
                                      double scale_x, double scale_y,
                                      double scale_z) {
  Marker box;
  box.type = Marker::CUBE;
  box.scale.x = scale_x;
  box.scale.y = scale_y;
  box.scale.z = scale_z;
  box.color.r = 0.5;
  box.color.g = 0.5;
  box.color.b = 0.5;
  box.color.a = 0.25;

  InteractiveMarkerControl control;
  control.markers.push_back(box);
  control.always_visible = true;
  control.interaction_mode = InteractiveMarkerControl::NONE;

  InteractiveMarker marker;
  marker.name = "box";
  marker.controls.push_back(control);
  marker.header.frame_id = base_frame_;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale = box.scale.x;

  return marker;
}

void Box3DRoiServer::Start() {
  InteractiveMarker box_marker = Box(0.5, 0, 1, 0.3, 0.3, 0.3);
  server_->insert(box_marker);
  roi_.transform.translation.x = 0.5;
  roi_.transform.translation.y = 0;
  roi_.transform.translation.z = 1;
  roi_.transform.rotation.w = 1;
  roi_.dimensions.x = 0.3;
  roi_.dimensions.y = 0.3;
  roi_.dimensions.z = 0.3;

  InteractiveMarker x_marker = Arrow("x", "pos", 0.5, 0, 1, 0.3, 0.3, 0.3);
  server_->insert(x_marker, boost::bind(&Box3DRoiServer::Feedback, this, _1));
  InteractiveMarker y_marker = Arrow("y", "pos", 0.5, 0, 1, 0.3, 0.3, 0.3);
  server_->insert(y_marker, boost::bind(&Box3DRoiServer::Feedback, this, _1));
  InteractiveMarker z_marker = Arrow("z", "pos", 0.5, 0, 1, 0.3, 0.3, 0.3);
  server_->insert(z_marker, boost::bind(&Box3DRoiServer::Feedback, this, _1));
  InteractiveMarker neg_x_marker = Arrow("x", "neg", 0.5, 0, 1, 0.3, 0.3, 0.3);
  server_->insert(neg_x_marker,
                  boost::bind(&Box3DRoiServer::Feedback, this, _1));
  InteractiveMarker neg_y_marker = Arrow("y", "neg", 0.5, 0, 1, 0.3, 0.3, 0.3);
  server_->insert(neg_y_marker,
                  boost::bind(&Box3DRoiServer::Feedback, this, _1));
  InteractiveMarker neg_z_marker = Arrow("z", "neg", 0.5, 0, 1, 0.3, 0.3, 0.3);
  server_->insert(neg_z_marker,
                  boost::bind(&Box3DRoiServer::Feedback, this, _1));

  server_->applyChanges();
}

void Box3DRoiServer::Stop() {
  server_->erase("box");
  server_->erase("pos_x");
  server_->erase("pos_y");
  server_->erase("pos_z");
  server_->erase("neg_x");
  server_->erase("neg_y");
  server_->erase("neg_z");
  server_->applyChanges();
}

InteractiveMarker Box3DRoiServer::Arrow(const std::string& dim,
                                        const std::string& polarity,
                                        double box_x, double box_y,
                                        double box_z, double box_scale_x,
                                        double box_scale_y,
                                        double box_scale_z) {
  Marker arrow;
  arrow.type = Marker::ARROW;
  arrow.scale.x = 0.05;
  arrow.scale.y = 0.025;
  arrow.scale.z = 0.025;
  if (dim == "x") {
    arrow.color.r = 1;
  } else if (dim == "y") {
    arrow.color.g = 1;
  } else if (dim == "z") {
    arrow.color.b = 1;
  }
  arrow.color.a = 1;

  double sign = 1;
  if (polarity == "neg") {
    sign = -1;
  }

  InteractiveMarkerControl control;
  control.orientation.w = 1;
  arrow.pose.orientation.w = 1;
  if (dim == "x") {
    if (sign == -1) {
      control.orientation.w = 0;
      control.orientation.z = 1;
    }
  } else if (dim == "y") {
    control.orientation.z = sign;
  } else if (dim == "z") {
    control.orientation.y = -sign;
  }
  arrow.pose.orientation = control.orientation;
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  control.markers.push_back(arrow);

  InteractiveMarker marker;
  marker.name = polarity + "_" + dim;
  marker.controls.push_back(control);
  marker.header.frame_id = base_frame_;
  marker.pose.position.x = box_x;
  marker.pose.position.y = box_y;
  marker.pose.position.z = box_z;
  marker.pose.orientation.w = 1;
  if (dim == "x") {
    marker.pose.position.x = box_x + sign * box_scale_x / 2 + sign * 0.05;
  } else if (dim == "y") {
    marker.pose.position.y = box_y + sign * box_scale_y / 2 + sign * 0.05;
  } else if (dim == "z") {
    marker.pose.position.z = box_z + sign * box_scale_z / 2 + sign * 0.05;
  }
  marker.scale = 0.05;
  return marker;
}

void Box3DRoiServer::Update(const std::string& dim, const std::string& polarity,
                            const geometry_msgs::Point& point) {
  InteractiveMarker old_box;
  server_->get("box", old_box);
  const geometry_msgs::Point& old_pos = old_box.pose.position;
  const geometry_msgs::Vector3& old_scale =
      old_box.controls[0].markers[0].scale;

  double sign = 1;
  if (polarity == "neg") {
    sign = -1;
  }

  double opposite_pos = 0;  // Position of opposite face.
  double input_pos = 0;     // Position requested by user.
  if (dim == "x") {
    input_pos = point.x - sign * 0.05;
    if (sign == 1) {
      opposite_pos = std::min(old_pos.x - old_scale.x / 2, input_pos - 0.01);
    } else {
      opposite_pos = std::max(old_pos.x + old_scale.x / 2, input_pos + 0.01);
    }
  } else if (dim == "y") {
    input_pos = point.y - sign * 0.05;
    if (sign == 1) {
      opposite_pos = std::min(old_pos.y - old_scale.y / 2, input_pos - 0.01);
    } else {
      opposite_pos = std::max(old_pos.y + old_scale.y / 2, input_pos + 0.01);
    }
  } else if (dim == "z") {
    input_pos = point.z - sign * 0.05;
    if (sign == 1) {
      opposite_pos = std::min(old_pos.z - old_scale.z / 2, input_pos - 0.01);
    } else {
      opposite_pos = std::max(old_pos.z + old_scale.z / 2, input_pos + 0.01);
    }
  }
  double new_scale = sign * (input_pos - opposite_pos);
  double new_pos = (opposite_pos + input_pos) / 2;

  // Update box
  InteractiveMarker box;
  double new_x = old_pos.x;
  double new_y = old_pos.y;
  double new_z = old_pos.z;
  double new_scale_x = old_scale.x;
  double new_scale_y = old_scale.y;
  double new_scale_z = old_scale.z;
  if (dim == "x") {
    new_x = new_pos;
    new_scale_x = new_scale;
  } else if (dim == "y") {
    new_y = new_pos;
    new_scale_y = new_scale;
  } else if (dim == "z") {
    new_z = new_pos;
    new_scale_z = new_scale;
  }
  box = Box(new_x, new_y, new_z, new_scale_x, new_scale_y, new_scale_z);
  server_->insert(box);

  // Update ROI
  roi_.transform.translation.x = new_x;
  roi_.transform.translation.y = new_y;
  roi_.transform.translation.z = new_z;
  roi_.dimensions.x = new_scale_x;
  roi_.dimensions.y = new_scale_y;
  roi_.dimensions.z = new_scale_z;

  // Update arrows
  InteractiveMarker pos_x = Arrow("x", "pos", new_x, new_y, new_z, new_scale_x,
                                  new_scale_y, new_scale_z);
  server_->setPose("pos_x", pos_x.pose, pos_x.header);
  InteractiveMarker neg_x = Arrow("x", "neg", new_x, new_y, new_z, new_scale_x,
                                  new_scale_y, new_scale_z);
  server_->setPose("neg_x", neg_x.pose, neg_x.header);

  InteractiveMarker pos_y = Arrow("y", "pos", new_x, new_y, new_z, new_scale_x,
                                  new_scale_y, new_scale_z);
  server_->setPose("pos_y", pos_y.pose, pos_y.header);
  InteractiveMarker neg_y = Arrow("y", "neg", new_x, new_y, new_z, new_scale_x,
                                  new_scale_y, new_scale_z);
  server_->setPose("neg_y", neg_y.pose, neg_y.header);

  InteractiveMarker pos_z = Arrow("z", "pos", new_x, new_y, new_z, new_scale_x,
                                  new_scale_y, new_scale_z);
  server_->setPose("pos_z", pos_z.pose, pos_z.header);
  InteractiveMarker neg_z = Arrow("z", "neg", new_x, new_y, new_z, new_scale_x,
                                  new_scale_y, new_scale_z);
  server_->setPose("neg_z", neg_z.pose, neg_z.header);

  server_->applyChanges();
}

void Box3DRoiServer::Feedback(
    const InteractiveMarkerFeedbackConstPtr& feedback) {
  const geometry_msgs::Point& point = feedback->pose.position;
  if (feedback->marker_name == "pos_x") {
    Update("x", "pos", point);
  } else if (feedback->marker_name == "pos_y") {
    Update("y", "pos", point);
  } else if (feedback->marker_name == "pos_z") {
    Update("z", "pos", point);
  } else if (feedback->marker_name == "neg_x") {
    Update("x", "neg", point);
  } else if (feedback->marker_name == "neg_y") {
    Update("y", "neg", point);
  } else if (feedback->marker_name == "neg_z") {
    Update("z", "neg", point);
  }
}

rapid_msgs::Roi3D Box3DRoiServer::roi() { return roi_; }

void Box3DRoiServer::set_base_frame(const std::string& base_frame) {
  base_frame_ = base_frame;
}
}  // namespace perception
}  // namespace rapid
