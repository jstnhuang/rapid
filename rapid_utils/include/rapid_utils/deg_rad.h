#ifndef _RAPID_UTILS_DEG_RAD_H_
#define _RAPID_UTILS_DEG_RAD_H_

// Types for degrees and radians.
//
// Most algorithms use radians, but simply taking in arguments as doubles can be
// confusing. Using these types, an algorithm can explicitly document that it
// takes in degrees or radians:
//
// head.PanTilt(rapid::Radians(0.707), rapid::Radians(0.5));
//
// We also allow users to implicitly convert degrees to radians. For example, if
// PanTilt takes in radians, users can nonetheless pass in degrees and have the
// conversion taken care of for them:
//
// head.PanTilt(rapid::Degrees(45), rapid::Degrees(30));

namespace rapid {
/// \brief Type for angles measured in degrees.
///
/// Allows algorithms to explicitly document what units it expects for angle
/// inputs.
class Degrees {
 public:
  /// \brief Constructor
  ///
  /// \param[in] degrees The angle expressed in degrees.
  explicit Degrees(double degrees);

  /// \returns The number of degrees.
  double value() const;

 private:
  double degrees_;
};

class Radians {
 public:
  /// \brief Constructor
  ///
  /// \param[in] radians The angle expressed in radians.
  explicit Radians(double radians);

  /// \brief Implicit converting constructor from degrees to radians.
  ///
  /// This allows algorithms that expect Radians to also accept Degrees without
  /// developers having to explicitly do the conversion. For example, if PanTilt
  /// expects arguments as Radians, both of these are correct:
  /// \code
  /// PanTilt(rapid::Radians(0.707), rapid::Radians(0.5));
  /// PanTilt(rapid::Degrees(45), rapid::Degrees(30));
  /// \endcode
  ///
  /// \param[in] degrees The angle expressed in degrees.
  Radians(const Degrees& degrees);

  /// \returns The number of radians.
  double value() const;

 private:
  double radians_;
};

}  // namespace rapid

#endif  // _RAPID_UTILS_DEG_RAD_H_
