#include "rapid_pbd/scene_db.h"

#include <string>
#include <vector>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

using boost::shared_ptr;
using sensor_msgs::PointCloud2;

namespace rapid {
namespace pbd {
SceneDb::SceneDb(mongodb_store::MessageStoreProxy* db) : db_(db) {}

std::string SceneDb::Insert(const PointCloud2& cloud) {
  std::string id = db_->insert(cloud);
  return id;
}

bool SceneDb::Get(const std::string& db_id, PointCloud2* cloud) const {
  std::vector<shared_ptr<PointCloud2> > results;
  bool success = db_->queryID(db_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Can't get scene with ID: \"%s\"", db_id.c_str());
    return false;
  }
  *cloud = *results[0];
  return true;
}

bool SceneDb::Delete(const std::string& db_id) {
  bool success = db_->deleteID(db_id);
  if (!success) {
    ROS_ERROR("Could not delete scene with ID \"%s\"", db_id.c_str());
  }
  return success;
}

}  // namespace pbd
}  // namespace rapid
