#include "rapid_manipulation/pick.h"

#include <string>
#include <vector>

#include "agile_grasp/Grasp.h"
#include "agile_grasp/Grasps.h"
#include "agile_grasp/FindGrasps.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
//#include "moveit_msgs/CollisionObject.h"
//#include "moveit_msgs/Grasp.h"
//#include "moveit_msgs/PlanningScene.h"
//#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
//#include "shape_msgs/SolidPrimitive.h"
//#include "shape_tools/solid_primitive_dims.h"

#include <Eigen/Dense>

#include "rapid_manipulation/arm.h"
#include "rapid_perception/scene.h"
#include "rapid_ros/service_client.h"
#include "rapid_utils/math.h"
#include "rapid_viz/markers.h"

using agile_grasp::FindGrasps;
using agile_grasp::Grasp;
using agile_grasp::Grasps;
using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
// using moveit_msgs::CollisionObject;
// using moveit_msgs::PlanningScene;
using rapid::perception::HSurface;
using rapid::perception::Object;
using rapid::perception::Scene;
// using shape_msgs::SolidPrimitive;
using std::string;
using std::vector;

namespace vmsgs = visualization_msgs;

namespace rapid {
namespace manipulation {
Picker::Picker(ArmInterface* arm, GripperInterface* gripper,
               rapid_ros::ServiceClientInterface<FindGrasps>* grasp_gen_client,
               rapid::viz::MarkerPub* marker_pub)
    :  // co_pub_(nh_.advertise<CollisionObject>("collision_object", 10)),
      // ps_pub_(nh_.advertise<PlanningScene>("planning_scene", 10)),
      grasp_gen_client_(grasp_gen_client),
      marker_pub_(marker_pub),
      scene_(),
      arm_(arm),
      gripper_(gripper) {
  // while (ps_pub_.getNumSubscribers() < 1) {
  //  ros::WallDuration sleep_t(0.5);
  //  sleep_t.sleep();
  //}
}

// void Picker::UpdatePlanningSceneTopic(const string& id,
//                                      const CollisionObject& obj) {
//  // Remove old object.
//  CollisionObject remove;
//  remove.id = id;
//  remove.operation = CollisionObject::REMOVE;
//  co_pub_.publish(remove);
//
//  PlanningScene ps;
//  ps.world.collision_objects.push_back(remove);
//  ps.is_diff = true;
//  // ps_pub_.publish(ps);
//
//  // Add updated object.
//  CollisionObject updated = obj;
//  updated.operation = CollisionObject::ADD;
//  ps.world.collision_objects.clear();
//  ps.world.collision_objects.push_back(updated);
//  ps.is_diff = true;
//  // ps_pub_.publish(ps);
//  co_pub_.publish(obj);
//}
//
// void Picker::UpdatePlanningScene(Scene& scene) {
//  scene_ = scene;
//
//  CollisionObject co;
//  co.header.stamp = ros::Time::now();
//  co.header.frame_id = "base_footprint";
//
//  // Update table.
//  const HSurface& table = scene.primary_surface();
//  co.operation = CollisionObject::ADD;
//  co.primitives.resize(1);
//  co.primitives[0].type = SolidPrimitive::BOX;
//  co.primitives[0].dimensions.resize(
//      shape_tools::SolidPrimitiveDimCount<SolidPrimitive::BOX>::value);
//  co.primitives[0].dimensions[SolidPrimitive::BOX_X] = table.scale().x;
//  co.primitives[0].dimensions[SolidPrimitive::BOX_Y] = table.scale().y;
//  co.primitives[0].dimensions[SolidPrimitive::BOX_Z] = table.scale().z;
//  co.primitive_poses.resize(1);
//  co.primitive_poses[0] = table.pose().pose;
//  UpdatePlanningSceneTopic("table", co);
//
//  // Update objects
//  const vector<Object>& objects = table.objects();
//  for (size_t i = 0; i < objects.size(); ++i) {
//    const Object& object = objects[i];
//    co.id = object.name();
//    cout << "Adding object " << co.id << " to planning scene." << endl;
//    co.operation = CollisionObject::ADD;
//    co.primitives[0].type = SolidPrimitive::BOX;
//    co.primitives[0].dimensions[SolidPrimitive::BOX_X] = object.scale().x;
//    co.primitives[0].dimensions[SolidPrimitive::BOX_Y] = object.scale().y;
//    co.primitives[0].dimensions[SolidPrimitive::BOX_Z] = object.scale().z;
//    co.header = object.pose().header;
//    co.header.stamp = ros::Time::now();
//    co.primitive_poses[0] = object.pose().pose;
//    UpdatePlanningSceneTopic(object.name(), co);
//  }
//}

bool Picker::Pick(const Object& obj, double max_effort) {
  // Use agile_grasp to find some grasp poses.
  Grasps grasps_response;
  bool success = GenerateGrasps(obj, &grasps_response);
  if (!success) {
    ROS_ERROR("Grasp generation service failed.");
    return false;
  }
  vector<Grasp>& grasps = grasps_response.grasps;
  if (grasps.size() == 0) {
    ROS_ERROR("Failed to find viable grasps.");
    return false;
  }

  std::string frame = grasps_response.header.frame_id;
  if (frame == "") {
    // This must always be the camera frame passed into agile_grasp.
    frame = "/head_mount_kinect_rgb_optical_frame";
  }

  // TODO(jstn): better grasp selection
  std::random_shuffle(grasps.begin(), grasps.end());
  int num_grasps = 0;
  ros::param::param<int>("num_grasps", num_grasps, 10);

  bool grasped_object = false;
  for (size_t grasp_i = 0;
       grasp_i < std::min(static_cast<size_t>(num_grasps), grasps.size());
       ++grasp_i) {
    const Grasp& grasp = grasps[grasp_i];
    VisualizeGrasp(grasp, frame);

    PoseStamped grasp_pose;
    grasp_pose.header.frame_id = frame;
    grasp_pose.pose.position.x = grasp.surface_center.x;
    grasp_pose.pose.position.y = grasp.surface_center.y;
    grasp_pose.pose.position.z = grasp.surface_center.z;
    ComputeGraspOrientation(grasp.approach, grasp.axis,
                            &grasp_pose.pose.orientation);

    // Pre-grasp
    ROS_INFO("Attempting pre-grasp");
    success = MoveToPreGrasp(grasp_pose, grasp.approach);
    if (!success) {
      ROS_ERROR("Failed to move to pre-grasp position.");
      continue;
    }
    gripper_->Open();

    // Grasp
    ROS_INFO("Attempting grasp");
    success = MoveToGrasp(grasp_pose, grasp.approach);
    if (!success) {
      ROS_ERROR("Failed to execute grasp.");
      continue;
    }
    gripper_->Close();
    gripper_->set_held_object(obj);
    grasped_object = true;
    break;
  }

  if (!grasped_object) {
    ROS_ERROR("Failed to grasp object.");
    return false;
  }

  ROS_INFO("Pick succeeded");
  return true;
}

bool Picker::GenerateGrasps(const rapid::perception::Object& obj,
                            agile_grasp::Grasps* grasps) {
  FindGrasps grasps_srv;
  pcl::PointCloud<pcl::PointXYZ> colorless;
  pcl::copyPointCloud(*obj.GetCloud(), colorless);
  pcl::toROSMsg(colorless, grasps_srv.request.object);
  bool success = grasp_gen_client_->call(grasps_srv);
  if (!success) {
    return false;
  }
  *grasps = grasps_srv.response.grasps;
  return true;
}

bool Picker::MoveToPreGrasp(const PoseStamped& grasp_pose,
                            const Vector3& grasp_approach) {
  PoseStamped pre_grasp = grasp_pose;
  double pregrasp_distance = 0;
  ros::param::param<double>("pregrasp_distance", pregrasp_distance, 0.3);
  double scale_factor = pregrasp_distance / rapid::utils::Norm(grasp_approach);
  pre_grasp.pose.position.x -= scale_factor * grasp_approach.x;
  pre_grasp.pose.position.y -= scale_factor * grasp_approach.y;
  pre_grasp.pose.position.z -= scale_factor * grasp_approach.z;
  return arm_->MoveToPoseGoal(pre_grasp);
}

bool Picker::MoveToGrasp(const geometry_msgs::PoseStamped& grasp_pose,
                         const geometry_msgs::Vector3& grasp_approach) {
  PoseStamped final_grasp_pose = grasp_pose;
  double grasp_distance = 0;  // Distance from the object during the grasp.
  ros::param::param<double>("grasp_distance", grasp_distance, 0.13);
  double scale_factor = grasp_distance / rapid::utils::Norm(grasp_approach);
  final_grasp_pose.pose.position.x -= scale_factor * grasp_approach.x;
  final_grasp_pose.pose.position.y -= scale_factor * grasp_approach.y;
  final_grasp_pose.pose.position.z -= scale_factor * grasp_approach.z;
  return arm_->MoveToPoseGoal(final_grasp_pose);
}

void Picker::VisualizeGrasp(const Grasp& grasp, const string& frame) {
  Point surface_center;
  surface_center.x = grasp.surface_center.x;
  surface_center.y = grasp.surface_center.y;
  surface_center.z = grasp.surface_center.z;

  rapid::viz::Marker m = rapid::viz::Marker::Vector(
      marker_pub_, frame, surface_center, grasp.approach);
  m.SetNamespace("grasp_approach");
  m.SetColor(1, 0, 0, 0.5);
  m.Publish();

  m = rapid::viz::Marker::Vector(marker_pub_, frame, surface_center,
                                 grasp.axis);
  m.SetNamespace("grasp_axis");
  m.SetColor(0, 1, 0, 0.5);
  m.Publish();
}

void ComputeGraspOrientation(const geometry_msgs::Vector3& approach,
                             const geometry_msgs::Vector3& axis,
                             geometry_msgs::Quaternion* orientation) {
  // Not sure if grasp.axis and grasp.approach are always orthogonal, so
  // compute y = x cross z, then z = x cross y.
  Eigen::Vector3d e_approach;
  e_approach << approach.x, approach.y, approach.z;
  Eigen::Vector3d e_axis;
  e_axis << axis.x, axis.y, axis.z;
  Eigen::Vector3d grasp_y;
  grasp_y = e_axis.cross(e_approach);
  e_axis = e_approach.cross(grasp_y);

  Eigen::Matrix3d rotation;
  rotation.col(0) = e_approach;
  rotation.col(1) = grasp_y;
  rotation.col(2) = e_axis;

  Eigen::Quaterniond eigen_q(rotation);
  eigen_q.normalize();
  geometry_msgs::Quaternion grasp_q;
  orientation->w = eigen_q.w();
  orientation->x = eigen_q.x();
  orientation->y = eigen_q.y();
  orientation->z = eigen_q.z();
}
}  // namespace manipulation
}  // namespace rapid
