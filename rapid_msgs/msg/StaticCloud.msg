# Represents a point cloud with the robot transformed saved.
string name
sensor_msgs/PointCloud2 cloud
string parent_frame_id # The base frame, e.g., "base_footprint"
geometry_msgs/Transform base_to_camera # Transform from parent_frame_id to cloud.header.frame_id
