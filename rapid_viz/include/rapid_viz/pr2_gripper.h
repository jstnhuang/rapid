#ifndef _RAPID_VIZ_PR2_GRIPPER_H_
#define _RAPID_VIZ_PR2_GRIPPER_H_

#include <math.h>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "interactive_markers/interactive_marker_server.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/InteractiveMarker.h"

namespace rapid {
namespace viz {
static const char* PR2_GRIPPER_PALM_URI =
    "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";
static const char* PR2_GRIPPER_FINGER_URI =
    "package://pr2_description/meshes/gripper_v0/l_finger.dae";
static const char* PR2_GRIPPER_FINGERTIP_URI =
    "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
static const double PR2_GRIPPER_MAX_RADIANS = 28.0 * M_PI / 180.0;

// Create an interactive marker representing a PR2 gripper.
// openness is between 0 and 1, with 1 being as open as possible.
// Leave color as all zeros to use the PR2 mesh color, otherwise it will be
// colored as given.
visualization_msgs::InteractiveMarker Pr2GripperMarker(
    const std::string& name, const geometry_msgs::PoseStamped& ps,
    double openness);
visualization_msgs::InteractiveMarker Pr2GripperMarker(
    const std::string& name, const geometry_msgs::PoseStamped& ps,
    double openness, std_msgs::ColorRGBA& color);
}  // namespace viz
}  // namespace rapid

#endif  // _RAPID_VIZ_PR2_GRIPPER_H_
