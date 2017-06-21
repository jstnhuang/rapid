#include <string>

#include "mongodb_store/message_store.h"
#include "rapid_pbd/program_db.h"
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

  ros::NodeHandle nh;
  mongodb_store::MessageStoreProxy proxy(nh, pbd::kMongoSceneCollectionName,
                                         pbd::kMongoDbName);
  pbd::SceneDb scene_db(&proxy);

  rapid::pbd::SurfaceSegmentationAction action(topic, scene_db);
  action.Start();
  ros::spin();
  return 0;
}
