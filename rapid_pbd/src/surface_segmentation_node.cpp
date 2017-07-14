#include <string>

#include "mongodb_store/message_store.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/surface_segmentation_action.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_segmentation_action_node");
  if (argc < 2) {
    ROS_ERROR("Must supply topic as arg");
    return 1;
  }
  std::string topic(argv[1]);

  std::string robot("");
  bool is_robot_specified = ros::param::get("robot", robot);
  if (!is_robot_specified) {
    ROS_ERROR("robot param must be specified.");
    return 1;
  }

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  pbd::RobotConfig* robot_config;
  if (robot == "pr2") {
    robot_config = new pbd::Pr2RobotConfig(robot_model_loader.getModel());
  } else if (robot == "fetch") {
    ROS_ERROR("Unsupported robot \"%s\"", robot.c_str());
    return 1;
  } else {
    ROS_ERROR("Unsupported robot \"%s\"", robot.c_str());
    return 1;
  }

  ros::NodeHandle nh;
  mongodb_store::MessageStoreProxy proxy(nh, pbd::kMongoSceneCollectionName,
                                         pbd::kMongoDbName);
  pbd::SceneDb scene_db(&proxy);

  rapid::pbd::SurfaceSegmentationAction action(topic, scene_db, *robot_config);
  action.Start();
  ros::spin();
  return 0;
}
