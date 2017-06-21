#ifndef _RAPID_PBD_SCENE_DB_H_
#define _RAPID_PBD_SCENE_DB_H_

#include <string>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_pbd/db_names.h"

namespace rapid {
namespace pbd {
class SceneDb {
 public:
  explicit SceneDb(mongodb_store::MessageStoreProxy* db);

  std::string Insert(const sensor_msgs::PointCloud2& cloud);
  bool Get(const std::string& db_id, sensor_msgs::PointCloud2* cloud) const;
  bool Delete(const std::string& db_id);

 private:
  mongodb_store::MessageStoreProxy* db_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_SCENE_DB_H_
