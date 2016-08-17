#ifndef _RAPID_UTILS_RANDOM_H_
#define _RAPID_UTILS_RANDOM_H_

namespace rapid {
namespace utils {
class RandomNumberInterface {
 public:
  virtual ~RandomNumberInterface() {}
  // Generate a pseudo-random uniformly distributed number in the range [0,
  // RAND_MAX] (same API as cstdlib's rand()).
  virtual int rand() const = 0;
};

class RandomNumber : public RandomNumberInterface {
 public:
  RandomNumber();
  int rand() const;
};

static RandomNumber BuildRandomNumber();

class MockRandomNumber : public RandomNumberInterface {
 public:
  MockRandomNumber();
  int rand() const;

  // Set what rand() will return next.
  void set_rand(int val);

 private:
  int number_;
};

}  // namespace utils
}  // namespace rapid

#endif  // _RAPID_UTILS_RANDOM_H_
