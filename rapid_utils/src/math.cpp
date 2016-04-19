#include "rapid_utils/math.h"

#include <math.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace rapid {
namespace utils {
bool AabbXYIntersect(const geometry_msgs::Point& pos1,
                     const geometry_msgs::Vector3& scale1,
                     const geometry_msgs::Point& pos2,
                     const geometry_msgs::Vector3& scale2) {
  double obj1_start_x = pos1.x - scale1.x / 2;
  double obj1_end_x = pos1.x + scale1.x / 2;
  double obj1_start_y = pos1.y - scale1.y / 2;
  double obj1_end_y = pos1.y + scale1.y / 2;
  double obj2_start_x = pos2.x - scale2.x / 2;
  double obj2_end_x = pos2.x + scale2.x / 2;
  double obj2_start_y = pos2.y - scale2.y / 2;
  double obj2_end_y = pos2.y + scale2.y / 2;
  return IntervalsIntersect(obj1_start_x, obj1_end_x, obj2_start_x,
                            obj2_end_x) &&
         IntervalsIntersect(obj1_start_y, obj1_end_y, obj2_start_y, obj2_end_y);
}

bool IntervalsIntersect(double start1, double end1, double start2,
                        double end2) {
  if (start1 < start2) {
    return end1 >= start2;
  } else if (start1 >= start2 && start1 < end2) {
    return true;
  } else {  // start1 > end2
    return false;
  }
}

double Norm(const geometry_msgs::Vector3& v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
}  //  namespace utils
}  // namespace rapid
