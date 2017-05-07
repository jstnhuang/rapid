#include "rapid_pbd/program_db.h"

#include <vector>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"

#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/ProgramInfo.h"
#include "rapid_pbd_msgs/ProgramInfoList.h"

using boost::shared_ptr;
using rapid_pbd_msgs::Program;
using rapid_pbd_msgs::ProgramInfo;
using rapid_pbd_msgs::ProgramInfoList;
using std::pair;
using std::vector;

namespace rapid {
namespace pbd {
ProgramDb::ProgramDb(mongodb_store::MessageStoreProxy db,
                     const ros::Publisher& list_pub)
    : db_(db), list_pub_(list_pub) {}

void ProgramDb::Start() { PublishList(); }

void ProgramDb::Insert(const rapid_pbd_msgs::Program& program) {
  std::string id = db_.insert(program);
  ROS_INFO("Inserted program \"%s\" with ID %s", program.name.c_str(),
           id.c_str());
  PublishList();
}

void ProgramDb::Update(const std::string& db_id,
                       const rapid_pbd_msgs::Program& program) {}

void ProgramDb::GetProgramById(const std::string& db_id) {}

void ProgramDb::Delete(const std::string& db_id) {
  bool success = db_.deleteID(db_id);

  if (success) {
    ROS_INFO("Deleted program with ID %s", db_id.c_str());
    PublishList();
  } else {
    ROS_ERROR("Could not delete program with ID \"%s\"", db_id.c_str());
  }
}

void ProgramDb::PublishList() {
  vector<pair<shared_ptr<Program>, mongo::BSONObj> > results;
  db_.query<Program>(results);
  ProgramInfoList msg;
  for (size_t i = 0; i < results.size(); ++i) {
    ProgramInfo info;
    info.name = results[i].first->name;
    info.db_id = results[i].second.getField("_id").OID().str();
    msg.programs.push_back(info);
  }
  list_pub_.publish(msg);
}
}  // namespace pbd
}  // namespace rapid
