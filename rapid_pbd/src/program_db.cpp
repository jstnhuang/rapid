#include "rapid_pbd/program_db.h"

#include <vector>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"

#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/ProgramList.h"

using boost::shared_ptr;
using rapid_pbd_msgs::Program;
using rapid_pbd_msgs::ProgramList;

namespace rapid {
namespace pbd {
ProgramDb::ProgramDb(mongodb_store::MessageStoreProxy db,
                     const ros::Publisher& list_pub)
    : db_(db), list_pub_(list_pub) {}

void ProgramDb::Start() { PublishList(); }

void ProgramDb::PublishList() {
  std::vector<shared_ptr<Program> > results;
  db_.query<Program>(results);
  ProgramList msg;
  for (size_t i = 0; i < results.size(); ++i) {
    msg.programs.push_back(*(results[i]));
  }
  list_pub_.publish(msg);
}
}  // namespace pbd
}  // namespace rapid
