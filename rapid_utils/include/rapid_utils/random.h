#ifndef _RAPID_UTILS_RANDOM_H_
#define _RAPID_UTILS_RANDOM_H_

namespace rapid {
namespace utils {
/// \brief Interface for random number generators.
class RandomNumberInterface {
 public:
  virtual ~RandomNumberInterface() {}
  /// \brief Generate a random number in [0, RAND_MAX]
  ///
  /// This is the same API as cstdlib's rand().
  ///
  /// \returns A uniformly distributed random number in the range [0, RAND_MAX].
  virtual int rand() const = 0;
};

/// \brief The cstdlib random number generator.
class RandomNumber : public RandomNumberInterface {
 public:
  RandomNumber();
  int rand() const;
};

/// \brief A mock random number generator for unit testing.
///
/// Usage:
/// \code
///   MockRandomNumber rng;
///   rng.set_rand(5);
///   rng.rand(); // Returns 5
/// \endcode
class MockRandomNumber : public RandomNumberInterface {
 public:
  MockRandomNumber();
  int rand() const;

  /// Set what rand() will return next.
  void set_rand(int val);

 private:
  int number_;
};
}  // namespace utils
}  // namespace rapid

#endif  // _RAPID_UTILS_RANDOM_H_
