#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace mpcc_cpp_virtual {

struct TrackReference {
  std::array<double, 2> center{};
  std::array<double, 2> tangent{};
  double width_left = 0;
  double width_right = 0;
};

class Track {
 public:
  static Track load(const std::string& path);
  TrackReference reference(double progress) const;
  double project(const std::array<double, 2>& point,
                 double previous_progress) const;
  double length() const { return length_; }

 private:
  std::array<double, 2> evaluate(double progress, bool derivative) const;

  double length_ = 0;
  int degree_ = 0;
  std::vector<double> knots_;
  std::vector<std::array<double, 2>> coefficients_;
  std::vector<double> width_progress_;
  std::vector<double> width_left_;
  std::vector<double> width_right_;
};

}  // namespace mpcc_cpp_virtual
