#ifndef _RAPID_PERCEPTION_SCENE_PARSING_H_
#define _RAPID_PERCEPTION_SCENE_PARSING_H_

#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_perception/scene.h"
#include "tf/transform_datatypes.h"

namespace rapid {
namespace perception {
struct SceneParseParams {
  // Boundaries of the box where we should search for the primary surface.
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;
};

struct HSurfaceParams {
  double distance_threshold;  // Max distance from fitted plane, in meters.
  double eps_angle;  // Max angle, in radians, off +z the HSurface can be.
};

struct ObjectParams {
  double distance_threshold;      // Distance to search for points to merge.
  double point_color_threshold;   // Max color difference between points.
  double region_color_threshold;  // Max color difference between clusters.
  double min_cluster_size;        // Minimum number of points per cluster.
};

struct ParseParams {
  SceneParseParams scene;
  HSurfaceParams hsurface;
  ObjectParams objects;
};

// Assumes the data is coming from a Kinect mounted on the PR2's head.
ParseParams Pr2Params();

// Parse a scene from the given point cloud, using default parameters.
// The point cloud must be transformed into a frame such that +z points up in
// the real world.
// Returns false if parsing failed.
bool ParseScene(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                const ParseParams& params, Scene* scene);

bool ParseHSurface(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                   const pcl::PointIndices::ConstPtr& indices,
                   const ParseParams& params, HSurface* surface);

bool ParseObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                  const pcl::PointIndices::ConstPtr& indices,
                  const ParseParams& params, std::vector<Object>* objects);

// A horizontal plane may cut across the entire scene, so we cluster the
// plane
// and call the largest cluster the tabletop.
// bool FindPrimarySurface(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
//                        pcl::PointIndices::Ptr inliers);

}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_SCENE_PARSING_H_
