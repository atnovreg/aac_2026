#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace mpcc_cpp_virtual {

class LapCounter {
 public:
  explicit LapCounter(double length) : length_(length) {
    if (!std::isfinite(length_) || length_ <= 0)
      throw std::invalid_argument("invalid lap length");
  }

  bool wrap(std::array<double, 7>& state) {
    if (!std::isfinite(state[6]) || state[6] < 0 || !std::isfinite(state[2]))
      throw std::invalid_argument("invalid virtual progress");
    bool crossed = false;
    while (state[6] >= length_) {
      state[6] -= length_;
      ++count_;
      crossed = true;
    }
    // Heading is a periodic coordinate. Keep its equivalent principal angle
    // when the caller resets the prediction after crossing the start line;
    // otherwise repeated laps eventually hit the artificial absolute bound.
    if (crossed) state[2] = std::remainder(state[2], 2*3.141592653589793238462643383279502884);
    return crossed;
  }

  int count() const { return count_; }
  double length() const { return length_; }
  double cumulative_progress(double current) const {
    return count_ * length_ + current;
  }

 private:
  double length_;
  int count_ = 0;
};

}  // namespace mpcc_cpp_virtual
