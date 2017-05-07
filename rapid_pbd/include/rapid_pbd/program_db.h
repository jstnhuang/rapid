#ifndef _RAPID_PBD_PROGRAM_DB_H_
#define _RAPID_PBD_PROGRAM_DB_H_

#include <string>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"

#include "rapid_pbd_msgs/Program.h"

namespace rapid {
namespace pbd {
static const char kMongoDbName[] = "rapid_pbd";
static const char kMongoCollectionName[] = "programs";
static const char kProgramListTopic[] = "rapid_pbd/program_list";

class ProgramDb {
 public:
  ProgramDb(mongodb_store::MessageStoreProxy db,
            const ros::Publisher& list_pub);

  // Publishes the first message.
  void Start();

  void Insert(const rapid_pbd_msgs::Program& program);
  void Update(const std::string& db_id, const rapid_pbd_msgs::Program& program);
  void GetProgramById(const std::string& db_id);
  void Delete(const std::string& db_id);

 private:
  mongodb_store::MessageStoreProxy db_;
  ros::Publisher list_pub_;

  void PublishList();
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_PROGRAM_DB_H_
