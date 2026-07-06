#include "cubic_spliner_2D.h"

#include <cmath>
#include <stdexcept>

CubicSpliner2D::CubicSpliner2D(const std::vector<double> &points_x,
                               const std::vector<double> &points_y)
    : points_x_(points_x), points_y_(points_y) {
  if (points_x_.size() != points_y_.size()) {
    throw std::invalid_argument("CubicSpliner2D x/y input sizes differ");
  }
  if (points_x_.size() < 2) {
    throw std::invalid_argument("CubicSpliner2D requires at least two points");
  }

  calculate_points_s(points_x_, points_y_);

  cubic_spliner_sx_ = std::make_shared<CubicSpliner>(points_s_, points_x_);
  cubic_spliner_sy_ = std::make_shared<CubicSpliner>(points_s_, points_y_);
}

void CubicSpliner2D::calculate_points_s(const std::vector<double> &points_x,
                                        const std::vector<double> &points_y) {
  points_s_.clear();
  points_s_.push_back(0.0);

  for (std::size_t i = 0; i + 1 < points_x.size(); ++i) {
    const double delta_s =
        std::hypot(points_x[i + 1] - points_x[i], points_y[i + 1] - points_y[i]);
    if (delta_s <= 1.0e-6) {
      throw std::invalid_argument("CubicSpliner2D input contains duplicate points");
    }
    points_s_.push_back(points_s_.back() + delta_s);
  }
}

std::tuple<double, double> CubicSpliner2D::calculate_position(double point_s) {
  const double point_x = cubic_spliner_sx_->calculate_zeroth_derivative(point_s);
  const double point_y = cubic_spliner_sy_->calculate_zeroth_derivative(point_s);
  return std::make_tuple(point_x, point_y);
}

double CubicSpliner2D::calculate_curvature(double point_s) {
  const double first_derivative_x =
      cubic_spliner_sx_->calculate_first_derivative(point_s);
  const double second_derivative_x =
      cubic_spliner_sx_->calculate_second_derivative(point_s);

  const double first_derivative_y =
      cubic_spliner_sy_->calculate_first_derivative(point_s);
  const double second_derivative_y =
      cubic_spliner_sy_->calculate_second_derivative(point_s);

  const double denominator =
      std::pow(first_derivative_x * first_derivative_x +
                   first_derivative_y * first_derivative_y,
               1.5);
  if (denominator <= 1.0e-9) {
    return 0.0;
  }

  return (second_derivative_y * first_derivative_x -
          second_derivative_x * first_derivative_y) /
         denominator;
}

double CubicSpliner2D::calculate_yaw(double point_s) {
  const double first_derivative_x =
      cubic_spliner_sx_->calculate_first_derivative(point_s);
  const double first_derivative_y =
      cubic_spliner_sy_->calculate_first_derivative(point_s);
  return std::atan2(first_derivative_y, first_derivative_x);
}
