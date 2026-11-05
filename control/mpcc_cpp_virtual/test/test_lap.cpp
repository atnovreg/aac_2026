#include "mpcc_cpp_virtual/lap.hpp"

#include <array>
#include <cmath>
#include <iostream>

int main() {
  mpcc_cpp_virtual::LapCounter laps(170.22736623711225);
  std::array<double, 7> state{1., 2., .1, 3., .01, .02, 170.24};
  const auto original = state;
  if (!laps.wrap(state) || laps.count() != 1) return 1;
  if (std::abs(state[6] - (original[6] - laps.length())) > 1e-12) return 2;
  for (int i = 0; i < 6; ++i)
    if (state[i] != original[i]) return 3;
  if (laps.wrap(state) || laps.count() != 1) return 4;
  if (std::abs(laps.cumulative_progress(state[6]) - original[6]) > 1e-12)
    return 5;
  // Physical orientation is periodic: repeated laps must not accumulate an
  // artificial absolute heading bound in the generated MPC state.
  mpcc_cpp_virtual::LapCounter periodic(10.);
  constexpr double pi = 3.141592653589793238462643383279502884;
  std::array<double, 7> rotated{1., 2., 4*pi+.25, 3., .01, .02, 10.1};
  const auto before = rotated;
  if (!periodic.wrap(rotated)) return 6;
  if (std::abs(rotated[2]-.25)>1e-12) return 7;
  if (std::abs(std::sin(rotated[2])-std::sin(before[2]))>1e-12 ||
      std::abs(std::cos(rotated[2])-std::cos(before[2]))>1e-12) return 8;
  for (int i : {0,1,3,4,5}) if (rotated[i]!=before[i]) return 9;
  rotated[2] = -4*pi-.25; rotated[6] = 10.1;
  if (!periodic.wrap(rotated) || std::abs(rotated[2]+.25)>1e-12) return 10;
  std::cout << "lap rollover preserves virtual XY/dynamics and cumulative progress\n";
}
