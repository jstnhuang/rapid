#include "rapid_ros/params.h"

#include <sstream>
#include <stdexcept>

#include "ros/ros.h"

namespace rapid {
namespace {
void ThrowParamNotFound(const std::string& param_name) {
  std::stringstream ss;
  ss << "Param \"" << param_name << "\" not found!";
  std::string message = ss.str();
  ROS_ERROR_STREAM(message);
  throw std::runtime_error(message);
}
}

std::string GetStringParamOrThrow(const std::string& name) {
  std::string result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

double GetDoubleParamOrThrow(const std::string& name) {
  double result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

float GetFloatParamOrThrow(const std::string& name) {
  float result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

int GetIntParamOrThrow(const std::string& name) {
  int result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

bool GetBoolParamOrThrow(const std::string& name) {
  bool result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

XmlRpc::XmlRpcValue GetXmlRpcParamOrThrow(const std::string& name) {
  XmlRpc::XmlRpcValue result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::vector<std::string> GetStringVectorParamOrThrow(const std::string& name) {
  std::vector<std::string> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::vector<double> GetDoubleVectorParamOrThrow(const std::string& name) {
  std::vector<double> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::vector<float> GetFloatVectorParamOrThrow(const std::string& name) {
  std::vector<float> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::vector<int> GetIntVectorParamOrThrow(const std::string& name) {
  std::vector<int> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::vector<bool> GetBoolVectorParamOrThrow(const std::string& name) {
  std::vector<bool> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::map<std::string, std::string> GetStringMapParamOrThrow(
    const std::string& name) {
  std::map<std::string, std::string> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::map<std::string, double> GetDoubleMapParamOrThrow(
    const std::string& name) {
  std::map<std::string, double> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::map<std::string, float> GetFloatMapParamOrThrow(const std::string& name) {
  std::map<std::string, float> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::map<std::string, int> GetIntMapParamOrThrow(const std::string& name) {
  std::map<std::string, int> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}

std::map<std::string, bool> GetBoolMapParamOrThrow(const std::string& name) {
  std::map<std::string, bool> result;
  if (!ros::param::get(name, result)) {
    ThrowParamNotFound(name);
  }
  return result;
}
}  // namespace rapid
