# All measurements are assumed to be in the robot's base frame
sensor_msgs/PointCloud2 cloud # The aligned point cloud of the landmark in the scene
geometry_msgs/Transform transform # Transform of the object relative to the provided input object
float64 error # Error score of the match, a lower score means a better match
