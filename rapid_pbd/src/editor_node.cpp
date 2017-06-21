#include "mongodb_store/message_store.h"
#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/db_names.h"
#include "rapid_pbd/editor.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd_msgs/GetEEPose.h"
#include "rapid_pbd_msgs/GetJointAngles.h"
#include "rapid_pbd_msgs/ProgramInfoList.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pbd_editor_node");
  ros::NodeHandle nh;

  // Build program DB.
  mongodb_store::MessageStoreProxy proxy(nh, pbd::kMongoProgramCollectionName,
                                         pbd::kMongoDbName);
  mongodb_store::MessageStoreProxy* scene_proxy =
      new mongodb_store::MessageStoreProxy(nh, pbd::kMongoSceneCollectionName,
                                           pbd::kMongoDbName);
  ros::Publisher program_list_pub =
      nh.advertise<rapid_pbd_msgs::ProgramInfoList>(pbd::kProgramListTopic, 1,
                                                    true);
  // Build DBs.
  pbd::ProgramDb db(nh, &proxy, program_list_pub);
  pbd::SceneDb scene_db(scene_proxy);

  // Build action clients.
  pbd::ActionClients action_clients;
  while (!action_clients.surface_segmentation_client.waitForServer(
             ros::Duration(5)) &&
         ros::ok()) {
    ROS_WARN("Waiting for surface segmentation server.");
  }

  // Build visualizer
  urdf::Model model;
  model.initParam("robot_description");
  robot_markers::Builder marker_builder(model);
  pbd::Visualizer visualizer(db, marker_builder);
  visualizer.Init();

  // Build editor.
  pbd::JointStateReader joint_state_reader;
  pbd::Editor editor(db, scene_db, joint_state_reader, visualizer,
                     &action_clients);
  editor.Start();

  ros::Subscriber editor_sub = nh.subscribe(pbd::kEditorEventsTopic, 10,
                                            &pbd::Editor::HandleEvent, &editor);
  ros::ServiceServer ee_pose_srv = nh.advertiseService(
      "get_ee_pose", &pbd::Editor::HandleGetEEPose, &editor);
  ros::ServiceServer joint_angles_srv = nh.advertiseService(
      "get_joint_angles", &pbd::Editor::HandleGetJointAngles, &editor);

  ROS_INFO("RapidPBD editor ready.");
  ros::spin();
  return 0;
}
