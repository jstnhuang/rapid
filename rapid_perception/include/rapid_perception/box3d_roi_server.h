#ifndef _RAPID_PERCEPTION_BOX3D_ROI_SERVER_H_
#define _RAPID_PERCEPTION_BOX3D_ROI_SERVER_H_

#include <string>

#include "geometry_msgs/Point.h"
#include "interactive_markers/interactive_marker_server.h"
#include "rapid_msgs/Roi3D.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"

namespace rapid {
namespace perception {
// An interactive marker server for labeling a 3D box region of interest.
class Box3DRoiServer {
 public:
  Box3DRoiServer(const std::string& topic);
  ~Box3DRoiServer();
  void Start();
  void Stop();
  rapid_msgs::Roi3D roi();

 private:
  void Feedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  visualization_msgs::InteractiveMarker Box(double x, double y, double z,
                                            double scale_x, double scale_y,
                                            double scale_z);
  visualization_msgs::InteractiveMarker Arrow(const std::string& dim,
                                              const std::string& polarity,
                                              double x, double y, double z,
                                              double scale_x, double scale_y,
                                              double scale_z);
  void Update(const std::string& dim, const std::string& polarity,
              const geometry_msgs::Point& point);
  interactive_markers::InteractiveMarkerServer* server_;
  rapid_msgs::Roi3D roi_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_BOX3D_ROI_SERVER_H_
