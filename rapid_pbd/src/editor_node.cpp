#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "mongodb_store/message_store.h"
#include "rapid_pbd/editor.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd_msgs/GetEEPose.h"
#include "rapid_pbd_msgs/GetJointAngles.h"
#include "rapid_pbd_msgs/ProgramInfoList.h"
#include "robot_state_publisher_latched/robot_state_publisher_latched.h"
#include "ros/ros.h"
#include "urdf/model.h"

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
  pbd::ProgramDb db(nh, &proxy, program_list_pub);

  // Build JointStates
  pbd::JointStateReader joint_state_reader;

  // Build VizServer
  urdf::Model model;
  model.initParam("robot_description");
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(model, tree);
  robot_state_publisher_latched::RobotStatePublisher robot_state_pub(tree);
  pbd::Visualizer visualizer(robot_state_pub);

  // Build editor.
  pbd::Editor editor(db, joint_state_reader, visualizer);
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
