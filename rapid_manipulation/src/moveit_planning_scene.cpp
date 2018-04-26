#include "rapid_manipulation/moveit_planning_scene.h"

#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/PlanningScene.h"
#include "shape_msgs/SolidPrimitive.h"

namespace rapid {

using shape_msgs::SolidPrimitive;

MoveItPlanningScene::MoveItPlanningScene()
    : topic_("/planning_scene"),
      nh_(),
      pub_(nh_.advertise<moveit_msgs::PlanningScene>(topic_, 1)) {
  Init();
}

MoveItPlanningScene::MoveItPlanningScene(const std::string& topic)
    : topic_(topic),
      nh_(),
      pub_(nh_.advertise<moveit_msgs::PlanningScene>(topic, 1)) {
  Init();
}

void MoveItPlanningScene::AddBoxObstacle(
    const std::string& name, const geometry_msgs::PoseStamped& pose_stamped,
    const geometry_msgs::Vector3& dims) {
  moveit_msgs::CollisionObject coll_obj;
  coll_obj.header.frame_id = pose_stamped.header.frame_id;
  coll_obj.id = name;
  SolidPrimitive box;
  box.type = SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[SolidPrimitive::BOX_X] = dims.x;
  box.dimensions[SolidPrimitive::BOX_Y] = dims.y;
  box.dimensions[SolidPrimitive::BOX_Z] = dims.z;
  coll_obj.primitives.push_back(box);
  coll_obj.primitive_poses.push_back(pose_stamped.pose);
  coll_obj.operation = moveit_msgs::CollisionObject::ADD;

  moveit_msgs::PlanningScene scene;
  scene.world.collision_objects.push_back(coll_obj);
  scene.is_diff = true;
  pub_.publish(scene);
}

void MoveItPlanningScene::RemoveObstacle(const std::string& name) {
  moveit_msgs::CollisionObject coll_obj;
  coll_obj.id = name;
  coll_obj.operation = moveit_msgs::CollisionObject::REMOVE;

  moveit_msgs::PlanningScene scene;
  scene.world.collision_objects.push_back(coll_obj);
  scene.is_diff = true;
  pub_.publish(scene);
}

void MoveItPlanningScene::Init() {
  ros::Time start = ros::Time::now();
  while (ros::ok() && pub_.getNumSubscribers() == 0) {
    ros::Time current = ros::Time::now();
    if (current - start > ros::Duration(5.0)) {
      ROS_ERROR("Failed to connect to MoveIt planning scene monitor!");
      return;
    }
    ros::spinOnce();
  }
}
}  // namespace rapid
