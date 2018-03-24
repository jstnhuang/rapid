# rapid_ros

rapid_ros contains utilities for common ROS features.

## ROS Parameters
When using ROS parameters, it is preferred to set all parameters in a config file and not specify any default values in the code.
Use functions like rapid::GetStringParamOrThrow, rapid::GetDoubleParamOrThrow, etc. to read parameters from the ROS parameter server.
These will throw a runtime error if the parameter does not exist.

~~~cpp
#include "rapid_ros/params.h"

double foo = rapid::GetDoubleParamOrThrow("foo");
~~~
