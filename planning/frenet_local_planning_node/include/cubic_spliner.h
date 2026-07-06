#ifndef CUBIC_SPLINER_H_
#define CUBIC_SPLINER_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

class CubicSpliner {
public:
  CubicSpliner(const std::vector<double> &points_x,
               const std::vector<double> &points_y);
  ~CubicSpliner() {}

  int number_of_points() const { return static_cast<int>(points_x_.size()); }
  double calculate_zeroth_derivative(double time);
  double calculate_first_derivative(double time);
  double calculate_second_derivative(double time);

protected:
  int find_index(double point_x);

private:
  std::vector<double> points_x_;
  std::vector<double> points_y_;

  std::vector<double> coefficient_a_;
  std::vector<double> coefficient_b_;
  std::vector<double> coefficient_c_;
  std::vector<double> coefficient_d_;
};

#endif // CUBIC_SPLINER_H_
