#ifndef _OBJECT_SEARCH_CLOUD_DATABASE_H_
#define _OBJECT_SEARCH_CLOUD_DATABASE_H_

#include <string>
#include <vector>

#include "ros/ros.h"

#include "rapid_msgs/StaticCloud.h"
#include "rapid_msgs/StaticCloudInfo.h"

namespace object_search {
class Database {
 public:
  Database(const std::string& db, const std::string& collection,
           const ros::ServiceClient& get, const ros::ServiceClient& list,
           const ros::ServiceClient& remove, const ros::ServiceClient& save);
  bool Get(const std::string& name, rapid_msgs::StaticCloud* cloud);
  void List(std::vector<rapid_msgs::StaticCloudInfo>* clouds);
  bool Remove(const std::string& id);
  std::string Save(const rapid_msgs::StaticCloud& cloud);

 private:
  std::string db_;
  std::string collection_;
  ros::ServiceClient get_;
  ros::ServiceClient list_;
  ros::ServiceClient remove_;
  ros::ServiceClient save_;
};
}  // namespace object_search

#endif  // _OBJECT_SEARCH_CLOUD_DATABASE_H_
