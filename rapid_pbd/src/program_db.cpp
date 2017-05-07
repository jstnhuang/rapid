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

void ProgramDb::PublishList() {
  vector<pair<shared_ptr<Program>, mongo::BSONObj> > results;
  db_.query<Program>(results);
  ProgramInfoList msg;
  for (size_t i = 0; i < results.size(); ++i) {
    ProgramInfo info;
    info.name = results[i].first->name;
    info.db_id = results[i].second.getField("_id").String();
    msg.programs.push_back(info);
  }
  list_pub_.publish(msg);
}
}  // namespace pbd
}  // namespace rapid
