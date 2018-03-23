#include "rapid_viz/six_dof_controls.h"

using visualization_msgs::InteractiveMarkerControl;

namespace rapid {
std::vector<InteractiveMarkerControl> FixedOrientationSixDofControls() {
  return SixDofControls(InteractiveMarkerControl::FIXED);
}

std::vector<InteractiveMarkerControl> FreeOrientationSixDofControls() {
  return SixDofControls(InteractiveMarkerControl::INHERIT);
}

std::vector<InteractiveMarkerControl> SixDofControls(uint8_t orientation_mode) {
  std::vector<InteractiveMarkerControl> controls;
  InteractiveMarkerControl control;
  control.orientation_mode = InteractiveMarkerControl::INHERIT;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);
  return controls;
}
}  // namespace rapid
