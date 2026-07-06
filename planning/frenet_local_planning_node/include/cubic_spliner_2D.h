#ifndef CUBIC_SPLINER_2D_H_
#define CUBIC_SPLINER_2D_H_

#include <memory>
#include <tuple>
#include <vector>

#include "cubic_spliner.h"

class CubicSpliner2D {
public:
  CubicSpliner2D(const std::vector<double> &points_x,
                 const std::vector<double> &points_y);
  ~CubicSpliner2D() {}

  const std::vector<double> &points_s() const { return points_s_; }

  std::tuple<double, double> calculate_position(double point_s);
  double calculate_curvature(double point_s);
  double calculate_yaw(double point_s);

protected:
  void calculate_points_s(const std::vector<double> &points_x,
                          const std::vector<double> &points_y);

private:
  std::vector<double> points_s_;
  std::vector<double> points_x_;
  std::vector<double> points_y_;

  std::shared_ptr<CubicSpliner> cubic_spliner_sx_;
  std::shared_ptr<CubicSpliner> cubic_spliner_sy_;
};

#endif // CUBIC_SPLINER_2D_H_
