#ifndef _RAPID_VIZ_AXES_MARKERS_H_
#define _RAPID_VIZ_AXES_MARKERS_H_

#include "visualization_msgs/MarkerArray.h"

namespace rapid {
/// \brief Generates an x/y/z axis marker given a pose.
///
/// The x-axis is red, y is green, z is blue.
///
/// \b Example usage:
/// \code
///   #include "rapid_viz/axes_markers.h"
///   visualization_msgs::MarkerArray axes_markers =
///     rapid::AxesMarkers("markers", "base_link", pose, 0.1);
/// \endcode
///
/// \param[in] ns The namespace for the markers.
/// \param[in] frame_id The frame_id of the pose to visualize.
/// \param[in] pose The pose to visualize.
/// \param[in] scale The length of each colored cylinder bar. The cylinder
///   diameters are 10% of scale.
///
/// \return The axes markers.
visualization_msgs::MarkerArray AxesMarkerArray(const std::string& ns,
                                                const std::string& frame_id,
                                                const geometry_msgs::Pose& pose,
                                                double scale);

}  // namespace rapid

#endif  // _RAPID_AXES_MARKERS_H_
