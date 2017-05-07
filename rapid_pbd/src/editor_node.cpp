#include "mongodb_store/message_store.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd_msgs/ProgramList.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pbd_editor_node");
  ros::NodeHandle nh;

  // Construct program DB.
  mongodb_store::MessageStoreProxy proxy(nh, pbd::kMongoCollectionName,
                                         pbd::kMongoDbName);
  ros::Publisher program_list_pub = nh.advertise<rapid_pbd_msgs::ProgramList>(
      pbd::kProgramListTopic, 1, true);
  pbd::ProgramDb db(proxy, program_list_pub);
  db.Start();

  ros::spin();
  return 0;
}
