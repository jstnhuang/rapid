#include <exception>

#include "Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_types.h"

namespace rapid {
template <typename T>
void get_xyz(const T& value, double* x, double* y, double* z) {
  throw std::invalid_argument("Unsupported type");
}

template <>
void get_xyz(const Eigen::Vector3f& value, double* x, double* y, double* z) {
  *x = value.x();
  *y = value.y();
  *z = value.z();
}

template <>
void get_xyz(const Eigen::Vector3d& value, double* x, double* y, double* z) {
  *x = value.x();
  *y = value.y();
  *z = value.z();
}

template <>
void get_xyz(const geometry_msgs::Point& value, double* x, double* y,
             double* z) {
  *x = value.x;
  *y = value.y;
  *z = value.z;
}

template <>
void get_xyz(const geometry_msgs::Vector3& value, double* x, double* y,
             double* z) {
  *x = value.x;
  *y = value.y;
  *z = value.z;
}

template <>
void get_xyz(const pcl::PointXYZ& value, double* x, double* y, double* z) {
  *x = value.x;
  *y = value.y;
  *z = value.z;
}

template <>
void get_xyz(const pcl::PointXYZRGB& value, double* x, double* y, double* z) {
  *x = value.x;
  *y = value.y;
  *z = value.z;
}

template <>
void get_xyz(const pcl::Normal& value, double* x, double* y, double* z) {
  *x = value.normal_x;
  *y = value.normal_y;
  *z = value.normal_z;
}

/// \brief Function to get the XYZ value from common ROS types.
///
/// These include:
/// - geometry_msgs: Point and Vector3
/// - pcl: PointXYZ, PointXYZRGB, Normal
/// See rapid_utils/vector3.hpp for the full list of types.
///
/// You can define your own template specialization of get_xyz for this to work
/// with other types.
///
/// \param[in] value A "vector3"-like object, see description above.
/// \throws std::invalid_argument if the type is not supported.
/// \returns An Eigen::Vector3d.
template <typename T>
Eigen::Vector3d AsVector3d(const T& value) {
  double x, y, z;
  get_xyz<T>(value, &x, &y, &z);
  Eigen::Vector3d result;
  result << x, y, z;
  return result;
}
}  // namespace rapid
