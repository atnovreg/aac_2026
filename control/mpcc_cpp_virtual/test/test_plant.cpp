#include "plant_step_10ms.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::ifstream fixture(argv[1]);
  if (!fixture) return 3;
  std::string line;
  std::getline(fixture, line);
  int count = 0;
  double max_error = 0;
  while (std::getline(fixture, line)) {
    std::stringstream stream(line);
    std::string field;
    std::vector<double> values;
    while (std::getline(stream, field, ',')) values.push_back(std::stod(field));
    if (values.size() != 17) return 4;
    std::array<double, 7> output{};
    const double* arguments[] = {values.data(), values.data() + 7};
    double* results[] = {output.data()};
    casadi_real work[slash_plant_step_10ms_SZ_W]{};
    if (slash_plant_step_10ms(arguments, results, nullptr, work, 0)) return 5;
    for (int i = 0; i < 7; ++i) {
      if (!std::isfinite(output[i])) return 6;
      max_error = std::max(max_error, std::abs(output[i] - values[i + 10]));
    }
    ++count;
  }
  if (count < 3 || max_error > 1e-8) {
    std::cerr << "plant mismatch count=" << count
              << " max_error=" << max_error << '\n';
    return 7;
  }
  std::cout << "plant rows=" << count << " max_abs_error=" << max_error << '\n';
}
