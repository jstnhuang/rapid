# rapid_utils

Small utility functions and classes.

Highlights:
- command_line.h and command_interface.h: A simple way to build command-line interfaces.
- pcl_conversions.h: Converts pcl::PointXYZ to geometry_msgs types and back.
- math.h: Interval intersection and axis-aligned bounding box intersection check (these will probably be moved in the future).
- random.h: Includes a random number generator interface and a mock random number generator, which is useful for testing.
- stochastic_universal_sampling.h: Stochastic Universal Sampling for CDFs
- vector3.hpp: Convert common Vector3-like types to an Eigen::Vector3d.
