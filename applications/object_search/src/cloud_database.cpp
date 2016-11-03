#include "object_search/cloud_database.h"

#include <string>

#include "rapid_msgs/GetStaticCloud.h"
#include "rapid_msgs/ListStaticClouds.h"
#include "rapid_msgs/RemoveStaticCloud.h"
#include "rapid_msgs/SaveStaticCloud.h"
#include "rapid_msgs/StaticCloud.h"
#include "rapid_msgs/StaticCloudInfo.h"

namespace object_search {
Database::Database(const std::string& db, const std::string& collection,
                   const ros::ServiceClient& get,
                   const ros::ServiceClient& list,
                   const ros::ServiceClient& remove,
                   const ros::ServiceClient& save)
    : db_(db),
      collection_(collection),
      get_(get),
      list_(list),
      remove_(remove),
      save_(save) {}

bool Database::Get(const std::string& name, rapid_msgs::StaticCloud* cloud) {
  rapid_msgs::GetStaticCloudRequest req;
  req.collection.db = db_;
  req.collection.collection = collection_;
  req.name = name;
  rapid_msgs::GetStaticCloudResponse res;
  bool success = get_.call(req, res);
  if (!success) {
    ROS_ERROR("Get call failed.");
    return false;
  }
  if (res.error != "") {
    ROS_ERROR("%s", res.error.c_str());
    return false;
  }
  *cloud = res.cloud;
  return true;
}

bool Database::GetById(const std::string& id, rapid_msgs::StaticCloud* cloud) {
  rapid_msgs::GetStaticCloudRequest req;
  req.collection.db = db_;
  req.collection.collection = collection_;
  req.id = id;
  rapid_msgs::GetStaticCloudResponse res;
  bool success = get_.call(req, res);
  if (!success) {
    ROS_ERROR("Get call failed.");
  }
  if (res.error != "") {
    ROS_ERROR("%s", res.error.c_str());
    return false;
  }
  *cloud = res.cloud;
  return true;
}

void Database::List(std::vector<rapid_msgs::StaticCloudInfo>* clouds) {
  rapid_msgs::ListStaticCloudsRequest req;
  req.collection.db = db_;
  req.collection.collection = collection_;
  rapid_msgs::ListStaticCloudsResponse res;
  bool success = list_.call(req, res);
  if (!success) {
    ROS_ERROR("List call failed.");
  }
  *clouds = res.clouds;
}

bool Database::Remove(const std::string& name) {
  rapid_msgs::RemoveStaticCloudRequest req;
  req.collection.db = db_;
  req.collection.collection = collection_;
  req.name = name;
  rapid_msgs::RemoveStaticCloudResponse res;
  bool success = remove_.call(req, res);
  if (!success) {
    ROS_ERROR("Remove call failed.");
  }
  if (res.error != "") {
    ROS_ERROR("%s", res.error.c_str());
    return false;
  }
  return true;
}

std::string Database::Save(const rapid_msgs::StaticCloud& cloud) {
  rapid_msgs::SaveStaticCloudRequest req;
  req.collection.db = db_;
  req.collection.collection = collection_;
  req.cloud = cloud;
  rapid_msgs::SaveStaticCloudResponse res;
  bool success = save_.call(req, res);
  if (!success) {
    ROS_ERROR("Save call failed.");
  }
  return res.id;
}
}  // namespace object_search
