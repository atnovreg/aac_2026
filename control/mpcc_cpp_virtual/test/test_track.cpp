#include "mpcc_cpp_virtual/track.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::vector<double> parse(const std::string& line) {
  std::vector<double> values;
  std::stringstream stream(line);
  std::string token;
  while (std::getline(stream, token, ',')) values.push_back(std::stod(token));
  return values;
}
}

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  const auto track = mpcc_cpp_virtual::Track::load(argv[1]);
  std::ifstream fixture(argv[2]);
  if (!fixture) return 3;
  std::string line;
  std::getline(fixture, line);
  int count = 0;
  double max_error = 0;
  while (std::getline(fixture, line)) {
    const auto expected = parse(line);
    if (expected.size() != 7) return 4;
    const auto actual = track.reference(expected[0]);
    const double fields[] = {actual.center[0], actual.center[1],
                             actual.tangent[0], actual.tangent[1],
                             actual.width_left, actual.width_right};
    for (int i = 0; i < 6; ++i) {
      if (!std::isfinite(fields[i])) return 5;
      max_error = std::max(max_error, std::abs(fields[i] - expected[i + 1]));
    }
    ++count;
  }
  if (count < 105 || max_error > 1e-8) {
    std::cerr << "track fixture mismatch: count=" << count
              << " max_error=" << max_error << '\n';
    return 6;
  }
  for (double progress : {0.02, 0.37 * track.length(),
                          track.length() - 0.02, track.length() + 0.02}) {
    const auto center = track.reference(progress).center;
    const double projected = track.project(center, progress - 0.03);
    if (std::abs(projected - progress) > 1e-6) return 9;
  }
  bool rejected = false;
  try { (void)mpcc_cpp_virtual::Track::load("/nonexistent/track_native.txt"); }
  catch (const std::exception&) { rejected = true; }
  if (!rejected) return 7;
  const auto corrupted = std::filesystem::temp_directory_path() /
                         "mpcc_corrupted_track_native.txt";
  {
    std::ofstream file(corrupted);
    file << "MPCC_TRACK_V1\n1 3 8 4 4\nnan\n";
  }
  rejected = false;
  try { (void)mpcc_cpp_virtual::Track::load(corrupted.string()); }
  catch (const std::exception&) { rejected = true; }
  std::filesystem::remove(corrupted);
  if (!rejected) return 8;
  std::cout << "track rows=" << count << " max_abs_error=" << max_error << '\n';
}
