#ifndef _RAPID_PERCEPTION_CLOUD_INDICES_H_
#define _RAPID_PERCEPTION_CLOUD_INDICES_H_

//#include "pcl/PointIndices.h"
//#include "pcl/point_cloud.h"
//#include "pcl/point_types.h"

#include "boost/shared_ptr.hpp"

namespace pcl {
class PointCloud<pcl::PointXYZRGB>;
class PointIndices;
typedef boost::shared_ptr<PointIndices> Ptr;
}  // namespace pcl

namespace rapid {
namespace perception {
// Bundles a pcl::PointCloud and pcl::PointIndices.
class CloudIndices {
 public:
 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  pcl::PointIndices::Ptr indices_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_CLOUD_INDICES_H_
