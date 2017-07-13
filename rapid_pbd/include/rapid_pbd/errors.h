#ifndef _RAPID_PBD_ERRORS_H_
#define _RAPID_PBD_ERRORS_H_

namespace rapid {
namespace pbd {
// Lists user-facing error messages.
namespace errors {
const char kNoLandmarksDetected[] = "No landmarks were detected.";
const char kNoLandmarksMatch[] = "Unable to find a matching object.";
}  // namespace errors
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ERRORS_H_
