#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
inline uint32_t offset_ns(double timestamp, int64_t base) {
  // Epoch nanoseconds stored in float64 have ~256 ns spacing. Subtract
  // in long double to preserve the integer header, and clamp rounding only.
  const long double delta = static_cast<long double>(timestamp) - base;
  if (!std::isfinite(timestamp) || delta < -256 || delta > 200000000)
    throw std::invalid_argument("Point timestamp is not absolute ns within a 200 ms scan");
  return delta < 0 ? 0 : static_cast<uint32_t>(std::llround(delta));
}
