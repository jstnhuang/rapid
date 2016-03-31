#ifndef _RAPID_VIZ_SHAPES_H_
#define _RAPID_VIZ_SHAPES_H_

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"

namespace rapid {
namespace viz {
void PublishMarker(const visualization_msgs::Marker& marker);

// Populates a Marker message with a bounding box.
void BoundingBoxMarker(const geometry_msgs::PoseStamped& pose,
                       const geometry_msgs::Vector3& scale,
                       visualization_msgs::Marker* marker);

void SetMarkerId(const std::string& ns, int id,
                 visualization_msgs::Marker* marker);

// Change the color of the given marker.
void SetMarkerColor(double r, double g, double b, double a,
                    visualization_msgs::Marker* marker);
}  // namespace viz
}  // namespace rapid
#endif  // _RAPID_VIZ_SHAPES_H_
