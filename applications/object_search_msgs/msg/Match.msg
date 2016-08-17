# All measurements are assumed to be in the robot's base frame
sensor_msgs/PointCloud2 cloud # The aligned point cloud of the landmark in the scene
geometry_msgs/Pose pose # Pose of the ROI bounding box around the landmark.
float64 error # Error score of the match, a lower score means a better match
