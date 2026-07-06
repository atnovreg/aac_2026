#include "cubic_spliner.h"

#include <stdexcept>

CubicSpliner::CubicSpliner(const std::vector<double> &points_x,
                           const std::vector<double> &points_y)
    : points_x_(points_x), points_y_(points_y) {
  if (points_x_.size() != points_y_.size()) {
    throw std::invalid_argument("CubicSpliner x/y input sizes differ");
  }
  if (points_x_.size() < 2) {
    throw std::invalid_argument("CubicSpliner requires at least two points");
  }

  std::vector<double> delta_x(points_x_.size() - 1);
  for (std::size_t i = 0; i + 1 < points_x_.size(); ++i) {
    delta_x[i] = points_x_[i + 1] - points_x_[i];
    if (delta_x[i] <= 0.0) {
      throw std::invalid_argument("CubicSpliner x input must be strictly increasing");
    }
  }

  coefficient_a_ = points_y;

  const int dimension = number_of_points();
  cv::Mat matrix_A = cv::Mat::zeros(dimension, dimension, CV_64F);
  cv::Mat matrix_B = cv::Mat::zeros(dimension, 1, CV_64F);

  matrix_A.at<double>(0, 0) = 1.0;
  matrix_A.at<double>(dimension - 1, dimension - 1) = 1.0;

  for (int i = 0; i < dimension - 2; ++i) {
    matrix_A.at<double>(i + 1, i) = delta_x[i];
    matrix_A.at<double>(i + 1, i + 1) = 2.0 * (delta_x[i] + delta_x[i + 1]);
    matrix_A.at<double>(i + 1, i + 2) = delta_x[i + 1];
    matrix_B.at<double>(i + 1, 0) =
        3.0 * (coefficient_a_[i + 2] - coefficient_a_[i + 1]) / delta_x[i + 1] -
        3.0 * (coefficient_a_[i + 1] - coefficient_a_[i]) / delta_x[i];
  }

  cv::Mat matrix_C;
  if (!cv::solve(matrix_A, matrix_B, matrix_C, cv::DECOMP_LU)) {
    throw std::runtime_error("CubicSpliner failed to solve coefficient matrix");
  }

  coefficient_c_.resize(points_x_.size());
  for (int i = 0; i < dimension; ++i) {
    coefficient_c_[i] = matrix_C.at<double>(i, 0);
  }

  for (std::size_t i = 0; i + 1 < points_x_.size(); ++i) {
    const double b =
        (coefficient_a_[i + 1] - coefficient_a_[i]) / delta_x[i] -
        delta_x[i] * (coefficient_c_[i + 1] + 2.0 * coefficient_c_[i]) / 3.0;
    const double d =
        (coefficient_c_[i + 1] - coefficient_c_[i]) / (3.0 * delta_x[i]);
    coefficient_b_.push_back(b);
    coefficient_d_.push_back(d);
  }
}

int CubicSpliner::find_index(double point_x) {
  if (point_x <= points_x_.front()) {
    return 0;
  }
  if (point_x >= points_x_.back()) {
    return static_cast<int>(points_x_.size()) - 2;
  }

  const int index =
      static_cast<int>(std::lower_bound(points_x_.begin(), points_x_.end(), point_x) -
                       points_x_.begin());
  return index == 0 ? 0 : index - 1;
}

double CubicSpliner::calculate_zeroth_derivative(double time) {
  if (time <= points_x_.front()) {
    time = points_x_.front();
  } else if (time >= points_x_.back()) {
    time = points_x_.back();
  }

  const int index = find_index(time);
  const double dx = time - points_x_[index];

  return coefficient_a_[index] + coefficient_b_[index] * dx +
         coefficient_c_[index] * std::pow(dx, 2) +
         coefficient_d_[index] * std::pow(dx, 3);
}

double CubicSpliner::calculate_first_derivative(double time) {
  if (time <= points_x_.front()) {
    time = points_x_.front();
  } else if (time >= points_x_.back()) {
    time = points_x_.back();
  }

  const int index = find_index(time);
  const double dx = time - points_x_[index];

  return coefficient_b_[index] + 2.0 * coefficient_c_[index] * dx +
         3.0 * coefficient_d_[index] * std::pow(dx, 2);
}

double CubicSpliner::calculate_second_derivative(double time) {
  if (time <= points_x_.front()) {
    time = points_x_.front();
  } else if (time >= points_x_.back()) {
    time = points_x_.back();
  }

  const int index = find_index(time);
  const double dx = time - points_x_[index];

  return 2.0 * coefficient_c_[index] + 6.0 * coefficient_d_[index] * dx;
}
