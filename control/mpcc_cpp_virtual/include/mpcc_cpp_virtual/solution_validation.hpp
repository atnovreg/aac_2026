#pragma once

#include <cmath>
#include <cstddef>

namespace mpcc_cpp_virtual {

inline bool valid_solution(const double* states, std::size_t state_count,
                           const double* increments, std::size_t increment_count,
                           const double* state_scale, std::size_t dimension = 10) {
  if (!states || !increments || !state_scale || dimension == 0 || state_count % dimension != 0)
    return false;
  for (std::size_t i = 0; i < state_count; ++i)
    if (!std::isfinite(states[i]) ||
        !std::isfinite(states[i] * state_scale[i % dimension])) return false;
  for (std::size_t i = 0; i < increment_count; ++i)
    if (!std::isfinite(increments[i])) return false;
  return true;
}

}  // namespace mpcc_cpp_virtual
