#include "rapid_utils/deg_rad.h"

#include <math.h>

namespace rapid {
Degrees::Degrees(double degrees) : degrees_(degrees) {}

double Degrees::value() const { return degrees_; }

Radians::Radians(double radians) : radians_(radians) {}

Radians::Radians(const Degrees& degrees)
    : radians_(degrees.value() * M_PI / 180) {}

double Radians::value() const { return radians_; }
}  // namespace rapid
