#ifndef _RAPID_MOVEIT_ERROR_CODE_H_
#define _RAPID_MOVEIT_ERROR_CODE_H_

#include <string>

#include "moveit/move_group_interface/move_group.h"
#include "moveit_msgs/MoveItErrorCodes.h"

namespace rapid {
/// \brief Returns true if the error code corresponds to success.
///
/// You can already use a MoveItErrorCode as a bool, but it can lead to
/// unintuitive code (true is success).
///
/// \code
/// MoveItErrorCode error = group.plan(...);
/// if (error) {
///   // This actually corresponds to success!
/// }
/// \endcode
///
/// Instead:
/// \code
/// MoveItErrorCode error = group.plan(...);
/// if (rapid::IsSuccess(error)) {
///   // More intuitive
/// }
/// \endcode
///
/// \param[in] error_code The MoveItErrorCode
/// \returns True if the error code is success, false otherwise.
bool IsSuccess(const moveit::planning_interface::MoveItErrorCode& error_code);

/// \brief Returns true if the error code corresponds to success.
///
/// \param[in] error_code The MoveItErrorCode
/// \returns True if the error code is success, false otherwise.
bool IsSuccess(const moveit_msgs::MoveItErrorCodes& error_code);

/// \brief Returns a string describing the given MoveIt error code.
///
/// \param[in] error_code The MoveItErrorCode
/// \returns A string corresponding to the name of the code in
///   moveit_msgs::MoveItErrorCodes.
std::string ErrorString(
    const moveit::planning_interface::MoveItErrorCode& error_code);

/// \brief Returns a string describing the given MoveIt error code.
///
/// \param[in] error_code The MoveItErrorCode
/// \returns A string corresponding to the name of the code in
///   moveit_msgs::MoveItErrorCodes.
std::string ErrorString(const moveit_msgs::MoveItErrorCodes& error_code);

}  // namespace pbi

#endif  // _PBI_MOVEIT_ERROR_CODE_H_
