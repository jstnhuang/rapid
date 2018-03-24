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

/// \brief Gets a string from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The string parameter
/// \throws std::runtime_error if the parameter does not exist.
std::string GetStringParamOrThrow(const std::string& name);

/// \brief Gets a double from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The double parameter
/// \throws std::runtime_error if the parameter does not exist.
double GetDoubleParamOrThrow(const std::string& name);

/// \brief Gets a float from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The float parameter
/// \throws std::runtime_error if the parameter does not exist.
float GetFloatParamOrThrow(const std::string& name);

/// \brief Gets an int from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The int parameter
/// \throws std::runtime_error if the parameter does not exist.
int GetIntParamOrThrow(const std::string& name);

/// \brief Gets a bool from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The bool parameter
/// \throws std::runtime_error if the parameter does not exist.
bool GetBoolParamOrThrow(const std::string& name);

/// \brief Gets an XML-RPC value from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The XML-RPC value
/// \throws std::runtime_error if the parameter does not exist.
XmlRpc::XmlRpcValue GetXmlRpcParamOrThrow(const std::string& name);

/// \brief Gets a vector of strings from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The vector of strings
/// \throws std::runtime_error if the parameter does not exist.
std::vector<std::string> GetStringVectorParamOrThrow(const std::string& name);

/// \brief Gets a vector of doubles from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The vector of doubles
/// \throws std::runtime_error if the parameter does not exist.
std::vector<double> GetDoubleVectorParamOrThrow(const std::string& name);

/// \brief Gets a vector of floats from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The vector of floats
/// \throws std::runtime_error if the parameter does not exist.
std::vector<float> GetFloatVectorParamOrThrow(const std::string& name);

/// \brief Gets a vector of ints from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The vector of ints
/// \throws std::runtime_error if the parameter does not exist.
std::vector<int> GetIntVectorParamOrThrow(const std::string& name);

/// \brief Gets a vector of bools from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The vector of bools
/// \throws std::runtime_error if the parameter does not exist.
std::vector<bool> GetBoolVectorParamOrThrow(const std::string& name);

/// \brief Gets a map from strings to string from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The map from strings to strings
/// \throws std::runtime_error if the parameter does not exist.
std::map<std::string, std::string> GetStringMapParamOrThrow(
    const std::string& name);

/// \brief Gets a map from strings to doubles from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The map from strings to doubles
/// \throws std::runtime_error if the parameter does not exist.
std::map<std::string, double> GetDoubleMapParamOrThrow(const std::string& name);

/// \brief Gets a map from strings to floats from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The map from strings to floats
/// \throws std::runtime_error if the parameter does not exist.
std::map<std::string, float> GetFloatMapParamOrThrow(const std::string& name);

/// \brief Gets a map from strings to ints from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The map from strings to ints
/// \throws std::runtime_error if the parameter does not exist.
std::map<std::string, int> GetIntMapParamOrThrow(const std::string& name);

/// \brief Gets a map from strings to bools from the parameter server.
///
/// \param[in] name The name of the parameter in the current node's namespace.
///
/// \returns The map from strings to bools
/// \throws std::runtime_error if the parameter does not exist.
std::map<std::string, bool> GetBoolMapParamOrThrow(const std::string& name);
}  // namespace rapid

#endif  // _RAPID_PARAMS_H_
