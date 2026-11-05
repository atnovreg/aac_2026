#include "timestamp.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  const int64_t base = 1788968406395562513LL;
  assert(offset_ns(static_cast<double>(base), base) <= 256);
  const auto dt = offset_ns(static_cast<double>(base + 100000000), base);
  assert(dt >= 99999744 && dt <= 100000256);
  assert(offset_ns(static_cast<double>(base - 128), base) == 0);
  for (const double t : {std::numeric_limits<double>::quiet_NaN(),
       static_cast<double>(base - 1000000), static_cast<double>(base + 1000000000), 1.0}) {
    bool rejected = false;
    try { offset_ns(t, base); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
  std::cout << "Timestamp validation passed\n";
}
