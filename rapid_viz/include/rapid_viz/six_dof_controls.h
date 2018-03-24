#ifndef _RAPID_VIZ_SIX_DOF_CONTROL_H_
#define _RAPID_VIZ_SIX_DOF_CONTROL_H_

#include <vector>
#include "visualization_msgs/InteractiveMarkerControl.h"

namespace rapid {
/// \brief Generates 6 DOF controls with fixed orientations.
///
/// A fixed orientation means that as the user rotates the interactive marker,
/// the direction of the controls stay fixed at their initial orientations.
///
/// Note that the scale of the controls is determined by the `scale` field of
/// your interactive marker.
///
/// \returns A list of InteractiveMarkerControls. These can be appended to an
///   InteractiveMarker to make it draggable and rotatable.
std::vector<visualization_msgs::InteractiveMarkerControl>
FixedOrientationSixDofControls();

/// \brief Generates 6 DOF controls with changeable orientations.
///
/// A "free" orientation means that as the user rotates the interactive marker,
/// the controls move along with the marker.
///
/// Note that the scale of the controls is determined by the `scale` field of
/// your interactive marker.
///
/// \returns A list of InteractiveMarkerControls. These can be appended to an
///   InteractiveMarker to make it draggable and rotatable.
std::vector<visualization_msgs::InteractiveMarkerControl>
FreeOrientationSixDofControls();

/// \brief (Somewhat internal). Generates a 6 DOF controls.
///
/// For clarity, use either FixedOrientationSixDofControls() or
/// FreeOrientationSixDofControls().
///
/// Note that the scale of the controls is determined by the `scale` field of
/// your interactive marker.
///
/// \returns A list of InteractiveMarkerControls. These can be appended to an
///   InteractiveMarker to make it draggable and rotatable.
std::vector<visualization_msgs::InteractiveMarkerControl> SixDofControls(
    uint8_t orientation_mode);
}  // namespace rapid

#endif  // _RAPID_VIZ_SIX_DOF_CONTROL_H_
