#include "rapid_perception/pose_estimation_heat_mapper.h"

#include <string>

#include "ros/ros.h"

namespace rapid {
namespace perception {
PoseEstimationHeatMapper::PoseEstimationHeatMapper()
    : debug_(false), heatmap_pub_() {}

void PoseEstimationHeatMapper::set_name(const std::string& val) { name_ = val; }

void PoseEstimationHeatMapper::set_debug(bool val) { debug_ = val; }

void PoseEstimationHeatMapper::set_heatmap_publisher(
    const ros::Publisher& pub) {
  heatmap_pub_ = pub;
}

std::string PoseEstimationHeatMapper::name() { return name_; }
}  // namespace perception
}  // namespace rapid
