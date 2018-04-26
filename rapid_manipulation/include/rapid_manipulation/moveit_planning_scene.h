#ifndef _RAPID_MOVEIT_PLANNING_SCENE_H_
#define _RAPID_MOVEIT_PLANNING_SCENE_H_

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"

namespace rapid {
/// \brief Simple interface to the MoveIt planning scene monitor.
///
/// Example usage:
/// \code
///   #include "rapid_manipulation/moveit_planning_scene.h"
///   rapid::MoveItPlanningScene planning_scene;
///   planning_scene.AddBoxObstacle("table", pose_stamped, dims);
///   ...
///   planning_scene.RemoveObstacle("table");
/// \endcode
class MoveItPlanningScene {
 public:
  /// \brief Default constructor, planning scene topic is "/planning_scene"
  MoveItPlanningScene();

  /// \brief Constructor that takes in a planning scene topic
  ///
  /// \param[in] topic The planning scene topic to publish to.
  explicit MoveItPlanningScene(const std::string& topic);

  /// \brief Adds a box-shaped obstacle to the planning scene.
  ///
  /// Adding an obstacle with the same name as an existing obstacle will replace
  /// it.
  ///
  /// \param[in] name The name of the obstacle.
  /// \param[in] pose_stamped The pose of the box. Origin is at the center of
  ///   the box.
  /// \param[in] dims The dimensions of the box.
  void AddBoxObstacle(const std::string& name,
                      const geometry_msgs::PoseStamped& pose_stamped,
                      const geometry_msgs::Vector3& dims);

  /// \brief Removes an obstacle from the planning scene.
  ///
  /// \param[in] name The name of the obstacle to remove.
  void RemoveObstacle(const std::string& name);

 private:
  void Init();

  std::string topic_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};
}  // namespace rapid

#endif  // _RAPID_MOVEIT_PLANNING_SCENE_H_
