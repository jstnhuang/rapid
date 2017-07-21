#ifndef _RAPID_PBD_PROGRAM_DB_H_
#define _RAPID_PBD_PROGRAM_DB_H_

#include <map>
#include <string>

#include "mongodb_store/message_store.h"
#include "rapid_pbd_msgs/Program.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_pbd/db_names.h"

namespace rapid {
namespace pbd {
static const char kProgramListTopic[] = "program_list";

class SceneDb {
 public:
  explicit SceneDb(mongodb_store::MessageStoreProxy* db);

  std::string Insert(const sensor_msgs::PointCloud2& cloud);
  bool Get(const std::string& db_id, sensor_msgs::PointCloud2* cloud) const;
  bool Delete(const std::string& db_id);

 private:
  mongodb_store::MessageStoreProxy* db_;
};

class ProgramDb {
 public:
  ProgramDb(const ros::NodeHandle& nh, mongodb_store::MessageStoreProxy* db,
            ros::Publisher* list_pub);

  // Publishes the first message.
  void Start();

  std::string Insert(const rapid_pbd_msgs::Program& program);
  void Update(const std::string& db_id, const rapid_pbd_msgs::Program& program);
  void StartPublishingProgramById(const std::string& db_id);
  bool Get(const std::string& db_id, rapid_pbd_msgs::Program* program) const;
  bool GetByName(const std::string& name,
                 rapid_pbd_msgs::Program* program) const;
  void Delete(const std::string& db_id);

 private:
  ros::NodeHandle nh_;
  mongodb_store::MessageStoreProxy* db_;
  ros::Publisher* list_pub_;
  std::map<std::string, ros::Publisher> program_pubs_;

  void PublishList();
  void PublishProgram(const std::string& db_id);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_PROGRAM_DB_H_
