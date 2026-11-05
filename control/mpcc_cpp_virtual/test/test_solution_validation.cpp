#include "mpcc_cpp_virtual/solution_validation.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
  std::array<double, 30> x{};
  std::array<double, 9> u{};
  const std::array<double, 10> scale{1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};
  if (!mpcc_cpp_virtual::valid_solution(x.data(), x.size(), u.data(),
                                        u.size(), scale.data())) return 1;
  x[29] = NAN;
  if (mpcc_cpp_virtual::valid_solution(x.data(), x.size(), u.data(),
                                       u.size(), scale.data())) return 2;
  x[29] = 0.;
  u[8] = INFINITY;
  if (mpcc_cpp_virtual::valid_solution(x.data(), x.size(), u.data(),
                                       u.size(), scale.data())) return 3;
  u[8] = 0.;
  std::array<double, 36> augmented{};
  std::array<double, 18> augmented_scale{};
  augmented_scale.fill(1.);
  if (!mpcc_cpp_virtual::valid_solution(augmented.data(), augmented.size(), u.data(),
                                       u.size(), augmented_scale.data(), 18)) return 4;
  augmented_scale[17] = std::numeric_limits<double>::max();
  augmented[35] = 2.;
  if (mpcc_cpp_virtual::valid_solution(augmented.data(), augmented.size(), u.data(),
                                      u.size(), augmented_scale.data(), 18)) return 5;
  std::cout << "rejects nonfinite future solver state and increment\n";
}
