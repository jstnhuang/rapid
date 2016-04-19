#ifndef _RAPID_UTILS_MATH_H_
#define _RAPID_UTILS_MATH_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace rapid {
namespace utils {
// Returns true if the axis-aligned bounding boxes intersect in both the X/Y
// dimensions.
bool AabbXYIntersect(const geometry_msgs::Point& pos1,
                     const geometry_msgs::Vector3& scale1,
                     const geometry_msgs::Point& pos2,
                     const geometry_msgs::Vector3& scale2);

// Returns true if the interval [start1, end1] intersects with [start2, end2].
bool IntervalsIntersect(double start1, double end1, double start2, double end2);

// Returns the length of the given vector.
double Norm(const geometry_msgs::Vector3& v);
}  //  namespace utils
}  // namespace rapid
#endif  //  _RAPID_UTILS_MATH_H_
