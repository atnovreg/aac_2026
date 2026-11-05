#include "mpcc_cpp_virtual/track.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace mpcc_cpp_virtual {
namespace {
void require(bool okay, const char* message) {
  if (!okay) throw std::runtime_error(message);
}
}

Track Track::load(const std::string& path) {
  std::ifstream input(path);
  require(bool(input), "cannot open native track");
  std::string version;
  input >> version;
  require(version == "MPCC_TRACK_V1", "unsupported native track version");
  Track result;
  std::size_t knot_count = 0, coefficient_count = 0, width_count = 0;
  input >> result.length_ >> result.degree_ >> knot_count >> coefficient_count
        >> width_count;
  require(bool(input) && std::isfinite(result.length_) && result.length_ > 0 &&
              result.degree_ == 3 && coefficient_count >= 4 &&
              knot_count == coefficient_count + 4 && width_count >= 4,
          "invalid native track header");
  result.knots_.resize(knot_count);
  result.coefficients_.resize(coefficient_count);
  result.width_progress_.resize(width_count);
  result.width_left_.resize(width_count);
  result.width_right_.resize(width_count);
  for (auto& knot : result.knots_) {
    input >> knot;
    require(bool(input) && std::isfinite(knot), "invalid track knot");
  }
  require(std::is_sorted(result.knots_.begin(), result.knots_.end()) &&
              result.knots_[3] <= 0 &&
              result.knots_[coefficient_count] >= result.length_,
          "invalid track knot domain");
  for (auto& coefficient : result.coefficients_) {
    input >> coefficient[0] >> coefficient[1];
    require(bool(input) && std::isfinite(coefficient[0]) &&
                std::isfinite(coefficient[1]), "invalid track coefficient");
  }
  for (std::size_t i = 0; i < width_count; ++i) {
    input >> result.width_progress_[i] >> result.width_left_[i]
          >> result.width_right_[i];
    require(bool(input) && std::isfinite(result.width_progress_[i]) &&
                std::isfinite(result.width_left_[i]) &&
                std::isfinite(result.width_right_[i]) &&
                result.width_left_[i] > 0 && result.width_right_[i] > 0,
            "invalid track width row");
    if (i) require(result.width_progress_[i] > result.width_progress_[i - 1],
                   "unordered track widths");
  }
  require(std::abs(result.width_progress_.front()) < 1e-12 &&
              result.width_progress_.back() < result.length_,
          "invalid width progress domain");
  std::string extra;
  require(!(input >> extra), "extra native track rows");
  return result;
}

std::array<double, 2> Track::evaluate(double progress, bool derivative) const {
  const int degree = derivative ? degree_ - 1 : degree_;
  const int offset = derivative ? 1 : 0;
  const int count = static_cast<int>(coefficients_.size()) - (derivative ? 1 : 0);
  const auto begin = knots_.begin() + offset;
  const int span = static_cast<int>(std::upper_bound(begin + degree,
      begin + count + 1, progress) - begin) - 1;
  std::array<std::array<double, 2>, 4> work{};
  for (int j = 0; j <= degree; ++j) {
    const int i = span - degree + j;
    if (!derivative) {
      work[j] = coefficients_[i];
    } else {
      const double denominator = knots_[i + degree_ + 1] - knots_[i + 1];
      require(denominator > 0, "degenerate track derivative knot");
      for (int axis = 0; axis < 2; ++axis)
        work[j][axis] = degree_ * (coefficients_[i + 1][axis] -
                                     coefficients_[i][axis]) / denominator;
    }
  }
  for (int r = 1; r <= degree; ++r) {
    for (int j = degree; j >= r; --j) {
      const int i = span - degree + j;
      const double left = begin[i];
      const double right = begin[i + degree - r + 1];
      const double denominator = right - left;
      require(denominator > 0, "degenerate track interpolation knot");
      const double alpha = (progress - left) / denominator;
      for (int axis = 0; axis < 2; ++axis)
        work[j][axis] = (1 - alpha) * work[j - 1][axis] + alpha * work[j][axis];
    }
  }
  return work[degree];
}

TrackReference Track::reference(double progress) const {
  require(std::isfinite(progress), "nonfinite track progress");
  double wrapped = std::fmod(progress, length_);
  if (wrapped < 0) wrapped += length_;
  const auto center = evaluate(wrapped, false);
  const auto derivative = evaluate(wrapped, true);
  const double speed = std::hypot(derivative[0], derivative[1]);
  require(std::isfinite(speed) && speed > 0, "invalid track tangent");

  const auto it = std::upper_bound(width_progress_.begin(),
                                   width_progress_.end(), wrapped);
  const std::size_t high = static_cast<std::size_t>(it - width_progress_.begin());
  const std::size_t low = high ? high - 1 : 0;
  const double s_low = width_progress_[low];
  const double s_high = high < width_progress_.size() ?
      width_progress_[high] : length_;
  const double alpha = (wrapped - s_low) / (s_high - s_low);
  const double left_high = high < width_progress_.size() ?
      width_left_[high] : width_left_[0];
  const double right_high = high < width_progress_.size() ?
      width_right_[high] : width_right_[0];
  return {center, {derivative[0] / speed, derivative[1] / speed},
          (1 - alpha) * width_left_[low] + alpha * left_high,
          (1 - alpha) * width_right_[low] + alpha * right_high};
}

double Track::project(const std::array<double, 2>& point,
                      double previous_progress) const {
  require(std::isfinite(point[0]) && std::isfinite(point[1]) &&
              std::isfinite(previous_progress), "invalid projection input");
  constexpr double window = .6;
  constexpr int samples = 12;
  constexpr double step = 2 * window / samples;
  const auto squared_distance = [&](double progress) {
    double wrapped = std::fmod(progress, length_);
    if (wrapped < 0) wrapped += length_;
    const auto center = evaluate(wrapped, false);
    return std::pow(center[0] - point[0], 2) +
           std::pow(center[1] - point[1], 2);
  };
  int best = 0;
  double best_distance = squared_distance(previous_progress - window);
  for (int i = 1; i <= samples; ++i) {
    const double value = squared_distance(previous_progress - window + i * step);
    if (value < best_distance) { best_distance = value; best = i; }
  }
  require(best > 0 && best < samples, "projection moved outside local search");
  double lo = previous_progress - window + (best - 1) * step;
  double hi = previous_progress - window + (best + 1) * step;
  constexpr double inv_phi = .6180339887498948482;
  double a = hi - inv_phi * (hi - lo);
  double b = lo + inv_phi * (hi - lo);
  double fa = squared_distance(a), fb = squared_distance(b);
  for (int i = 0; i < 42; ++i) {
    if (fa < fb) {
      hi = b;
      b = a;
      fb = fa;
      a = hi - inv_phi * (hi - lo);
      fa = squared_distance(a);
    } else {
      lo = a;
      a = b;
      fa = fb;
      b = lo + inv_phi * (hi - lo);
      fb = squared_distance(b);
    }
  }
  return (lo + hi) / 2;
}

}  // namespace mpcc_cpp_virtual
