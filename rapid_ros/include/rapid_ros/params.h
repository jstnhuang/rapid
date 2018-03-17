#ifndef _RAPID_PARAMS_H_
#define _RAPID_PARAMS_H_

#include <map>
#include <string>
#include <vector>

#include "XmlRpcValue.h"

namespace rapid {
// ---------------------------------------------------------------------------
// Functions to get a parameter from the ROS parameter server that throw
// std::runtime_error if the parameter is not found. We recommend applications
// explicitly list parameters in config files (i.e., in launch files or YAML
// files) rather than relying on default values set in code.

/// Gets a string param from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The string parameter
/// \throws std::runtime_error if the parameter does not exist.
std::string GetStringParamOrThrow(const std::string& name);

double GetDoubleParamOrThrow(const std::string& name);

float GetFloatParamOrThrow(const std::string& name);

int GetIntParamOrThrow(const std::string& name);

bool GetBoolParamOrThrow(const std::string& name);

XmlRpc::XmlRpcValue GetXmlRpcParamOrThrow(const std::string& name);

std::vector<std::string> GetStringVectorParamOrThrow(const std::string& name);

std::vector<double> GetDoubleVectorParamOrThrow(const std::string& name);

std::vector<float> GetFloatVectorParamOrThrow(const std::string& name);

std::vector<int> GetIntVectorParamOrThrow(const std::string& name);

std::vector<bool> GetBoolVectorParamOrThrow(const std::string& name);

std::map<std::string, std::string> GetStringMapParamOrThrow(
    const std::string& name);

std::map<std::string, double> GetDoubleMapParamOrThrow(const std::string& name);

std::map<std::string, float> GetFloatMapParamOrThrow(const std::string& name);

std::map<std::string, int> GetIntMapParamOrThrow(const std::string& name);

std::map<std::string, bool> GetBoolMapParamOrThrow(const std::string& name);
}  // namespace rapid

#endif  // _RAPID_PARAMS_H_
