#ifndef _OBJECT_SEARCH_OBJECT_SEARCH_NODE_H_
#define _OBJECT_SEARCH_OBJECT_SEARCH_NODE_H_

#include <string>
#include <vector>

#include "geometry_msgs/Transform.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_msgs/StaticCloud.h"

#include "object_search/cloud_database.h"
#include "object_search/commands.h"
#include "object_search_msgs/GetObjectInfo.h"
#include "object_search_msgs/Match.h"
#include "object_search_msgs/RecordObject.h"
#include "object_search_msgs/Search.h"
#include "object_search_msgs/SearchFromDb.h"

namespace object_search {
class ObjectSearchNode {
 public:
  ObjectSearchNode(const rapid::perception::PoseEstimator& estimator,
                   const RecordObjectCommand& record_object,
                   const Database& object_db);
  bool ServeGetObjectInfo(object_search_msgs::GetObjectInfoRequest& req,
                          object_search_msgs::GetObjectInfoResponse& resp);
  bool ServeRecordObject(object_search_msgs::RecordObjectRequest& req,
                         object_search_msgs::RecordObjectResponse& resp);
  bool ServeSearch(object_search_msgs::SearchRequest& req,
                   object_search_msgs::SearchResponse& resp);
  bool ServeSearchFromDb(object_search_msgs::SearchFromDbRequest& req,
                         object_search_msgs::SearchFromDbResponse& resp);

 private:
  void UpdateParams();
  void Search(const rapid_msgs::StaticCloud& scene,
              const rapid_msgs::StaticCloud& object, const bool is_tabletop,
              const double max_error, const int min_results,
              std::vector<object_search_msgs::Match>* matches);
  void Downsample(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);
  void TransformToBase(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in,
                       const std::string& parent_frame_id,
                       const geometry_msgs::Transform& base_to_camera,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);
  void ExtractTabletop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);
  void CropScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);

  tf::TransformListener tf_listener_;
  rapid::perception::PoseEstimator estimator_;
  RecordObjectCommand record_object_;
  Database object_db_;

  // Parameters
  // Voxelization
  double leaf_size_;

  // Scene cropping
  double min_x_;
  double min_y_;
  double min_z_;
  double max_x_;
  double max_y_;
  double max_z_;

  // Search
  double sample_ratio_;
  int max_samples_;
  double fitness_threshold_;
  double sigma_threshold_;
  double nms_radius_;
};
}  // namespace object_search

#endif  // _OBJECT_SEARCH_OBJECT_SEARCH_NODE_H_
