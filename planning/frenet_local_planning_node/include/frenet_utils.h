/*
 * frenet_utils.h
 *
 *  Created on: Apr 14, 2021
 *      Author: Daegyu Lee
 */
#ifndef FRENET_UTILS_H
#define FRENET_UTILS_H

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>

namespace frenet_utils
{
void GetParallelRefPath(const std::vector<double> &map_track_x_in,
                        const std::vector<double> &map_track_y_in,
                        double offset,
                        std::vector<double> &perp_x_out,
                        std::vector<double> &perp_y_out);

void DrawParallelPathPolygon(const std::vector<double> &x_parallel_in,
                             const std::vector<double> &y_parallel_in,
                             double parallel_in,
                             geometry_msgs::msg::PolygonStamped &ref_path_boundary_out,
                             double parallel_scale);

void DrawVehicleCollisionPolygon(const std::vector<double> &local_maptrack_x_in,
                                 const std::vector<double> &local_maptrack_y_in,
                                 geometry_msgs::msg::PolygonStamped &collision_boundary_out,
                                 double veh_width);

double CalculateProjDist(const geometry_msgs::msg::Point &point_in,
                         const nav_msgs::msg::Path &ref_path_in);

double CalculateSignedProjDist(const geometry_msgs::msg::Point &point_in,
                               const nav_msgs::msg::Path &ref_path_in);

bool isInside(const geometry_msgs::msg::PolygonStamped &polygon_in,
              const geometry_msgs::msg::Point32 &p);

} // namespace frenet_utils

#endif // FRENET_UTILS_H
