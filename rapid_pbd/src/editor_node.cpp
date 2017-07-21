#include "mongodb_store/message_store.h"
#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/db_names.h"
#include "rapid_pbd/editor.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd_msgs/ProgramInfoList.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pbd_editor_node");
  ros::NodeHandle nh;

  // Build robot config.
  std::string robot("");
  bool is_robot_specified = ros::param::get("robot", robot);
  if (!is_robot_specified) {
    ROS_ERROR("robot param must be specified.");
    return 1;
  }

  pbd::RobotConfig* robot_config;
  if (robot == "pr2") {
    robot_config = new pbd::Pr2RobotConfig();
  } else if (robot == "fetch") {
    robot_config = new pbd::FetchRobotConfig();
  } else {
    ROS_ERROR("Unsupported robot \"%s\"", robot.c_str());
    return 1;
  }

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
  pbd::ProgramDb db(nh, &proxy, &program_list_pub);
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
  pbd::Visualizer visualizer(scene_db, marker_builder, *robot_config);
  visualizer.Init();

  // Build editor.
  pbd::JointStateReader joint_state_reader;
  pbd::Editor editor(db, scene_db, joint_state_reader, visualizer,
                     &action_clients, *robot_config);
  editor.Start();

  ros::Subscriber editor_sub = nh.subscribe(pbd::kEditorEventsTopic, 10,
                                            &pbd::Editor::HandleEvent, &editor);

  ROS_INFO("RapidPBD editor ready.");
  ros::spin();
  if (robot_config) {
    delete robot_config;
  }
  return 0;
}
