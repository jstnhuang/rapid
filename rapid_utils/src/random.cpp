#include "rapid_utils/random.h"

#include <cstdlib>

namespace rapid {
namespace utils {
RandomNumber::RandomNumber() {}

int RandomNumber::rand() const { return ::rand(); }

MockRandomNumber::MockRandomNumber() : number_(0) {}

int MockRandomNumber::rand() const { return number_; }

void MockRandomNumber::set_rand(int val) { number_ = val; }
}  // namespace utils
}  // namespace rapid
