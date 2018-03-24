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
/// \brief An interactive marker for a box with draggable sides.
//
/// The box has a fixed pose but adjustable-length sides. After the user adjusts
/// the side lengths, you can access the sides via min_x, max_x, etc. Note that
/// the pose is not necessarily centered on or contained in the box. For
/// example, if the pose could be at the origin while min_x=0.05 and max_x =
/// 0.15. The box would appear to be centered at x=0.1.
///
/// Usage:
/// \code
///   DragBoxMarker box("left_hand", &im_server);
///   box.set_pose_stamped(ps);
///   box.set_min_x(-0.15);
///   box.set_max_x(0.15);
///   box.set_min_y(-0.15);
///   box.set_max_y(0.15);
///   box.set_min_z(-0.15);
///   box.set_max_z(0.15);
///   box.Show();
///
///   // Wait for user input, e.g., std::cin or waitKey
///   double min_x = box.min_x();
///   box.Hide();
/// \endcode
class DragBoxMarker {
 public:
  /// \brief Constructor
  ///
  /// \param[in] name The name of the this interactive marker.
  /// \param[in,out] im_server The interactive marker server to add this marker
  ///   to.
  DragBoxMarker(const std::string& name,
                interactive_markers::InteractiveMarkerServer* im_server);

  /// \brief Show the marker.
  void Show();

  /// \brief Erases the marker.
  void Hide();

  /// \brief Sets the marker's pose.
  void set_pose_stamped(const geometry_msgs::PoseStamped& pose_stamped);

  /// \brief Sets the minimum x value of the box.
  /// \param[in] min_x The minimum x value of the box.
  void set_min_x(double min_x);

  /// \brief Sets the maximum x value of the box.
  /// \param[in] max_x The maximum x value of the box.
  void set_max_x(double max_x);

  /// \brief Sets the minimum y value of the box.
  /// \param[in] min_y The minimum y value of the box.
  void set_min_y(double min_y);

  /// \brief Sets the maximum y value of the box.
  /// \param[in] max_y The maximum y value of the box.
  void set_max_y(double max_y);

  /// \brief Sets the minimum z value of the box.
  /// \param[in] min_z The minimum z value of the box.
  void set_min_z(double min_z);

  /// \brief Sets the maximum z value of the box.
  /// \param[in] max_x The maximum z value of the box.
  void set_max_z(double max_z);

  /// \returns The pose of the box.
  geometry_msgs::PoseStamped pose_stamped();
  /// \returns The minimum x value of the box.
  double min_x();
  /// \returns The maximum x value of the box.
  double max_x();
  /// \returns The minimum y value of the box.
  double min_y();
  /// \returns The maximum y value of the box.
  double max_y();
  /// \returns The minimum z value of the box.
  double min_z();
  /// \returns The maximum z value of the box.
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
