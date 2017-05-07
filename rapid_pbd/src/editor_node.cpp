#include "mongodb_store/message_store.h"
#include "rapid_pbd/editor.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd_msgs/ProgramInfoList.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pbd_editor_node");
  ros::NodeHandle nh;

  // Build program DB.
  mongodb_store::MessageStoreProxy proxy(nh, pbd::kMongoCollectionName,
                                         pbd::kMongoDbName);
  ros::Publisher program_list_pub =
      nh.advertise<rapid_pbd_msgs::ProgramInfoList>(pbd::kProgramListTopic, 1,
                                                    true);
  pbd::ProgramDb db(proxy, program_list_pub);

  // Build editor.
  pbd::Editor editor(db);
  editor.Start();

  ros::Subscriber editor_sub = nh.subscribe(pbd::kEditorEventsTopic, 10,
                                            &pbd::Editor::HandleEvent, &editor);

  ros::spin();
  return 0;
}
