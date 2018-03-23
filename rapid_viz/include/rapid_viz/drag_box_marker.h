#ifndef _RAPID_VIZ_DRAG_BOX_MARKER_H_
#define _RAPID_VIZ_DRAG_BOX_MARKER_H_

#include <string>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "interactive_markers/interactive_marker_server.h"
#include "transform_graph/graph.h"
#include "visualization_msgs/InteractiveMarker.h"

namespace rapid {
// An interactive marker for a box that has a fixed pose but adjustable-length
// sides. After the user adjusts the side lengths, you can access the sides via
// min_x, max_x, etc. Note that the pose is not necessarily centered on or
// contained in the box. For example, if the pose is at the origin and
// min_x=0.05 and max_x = 0.15, then the box is centered x=0.1
//
// Usage:
//   DragBoxMarker box("left_hand", &im_server);
//   box.set_pose_stamped(ps);
//   box.Show();
//
//   // Wait for user input, e.g., std::cin or waitKey
//   double min_x = box.min_x();
class DragBoxMarker {
 public:
  DragBoxMarker(const std::string& name,
                interactive_markers::InteractiveMarkerServer* im_server);

  void Show();

  void set_pose_stamped(const geometry_msgs::PoseStamped& pose_stamped);
  void set_min_x(double min_x);
  void set_max_x(double max_x);
  void set_min_y(double min_y);
  void set_max_y(double max_y);
  void set_min_z(double min_z);
  void set_max_z(double max_z);
  double min_x();
  double max_x();
  double min_y();
  double max_y();
  double min_z();
  double max_z();

 private:
  // Update the interactive markers, assuming that min/max_xyz have been set.
  void Update();
  // Computes min/max_xyz given a new arrow position
  void UpdateArrow(const std::string& axis, const std::string& polarity,
                   const geometry_msgs::Point& point);
  void Feedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void MakeBox(visualization_msgs::InteractiveMarker* box);
  void MakeArrow(const std::string& axis, const std::string& polarity,
                 visualization_msgs::InteractiveMarker* arrow);

  std::string name_;
  interactive_markers::InteractiveMarkerServer* im_server_;

  geometry_msgs::PoseStamped pose_stamped_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;

  transform_graph::Graph tf_graph_;
};

}  // namespace rapid

#endif  // _RAPID_VIZ_DRAG_BOX_MARKER_H_
