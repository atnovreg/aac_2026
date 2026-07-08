#include "parking_map_publisher/parking_map_publisher.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace
{
constexpr const char * kTrackCsvName = "track_texture_lines_wide.csv";
constexpr const char * kParkingCsvName = "parking_lot_lines.csv";
constexpr double kEpsilon = 1.0e-9;

bool isCsvFile(const fs::path & path)
{
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return ext == ".csv";
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

int clampOccupancyValue(int value)
{
  if (value < 0) {
    return -1;
  }
  return std::max(0, std::min(100, value));
}
}  // namespace

ParkingMapPublisher::ParkingMapPublisher(const std::string & node_name)
: Node(node_name)
{
  declare_parameter<std::string>("track_csv_path", "");
  declare_parameter<std::string>("parking_csv_path", "");
  declare_parameter<std::string>("map_topic", "/parking_map");
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<double>("publish_rate", 1.0);
  declare_parameter<double>("map_x_min", -5.5);
  declare_parameter<double>("map_x_max", 5.5);
  declare_parameter<double>("map_y_min", -8.0);
  declare_parameter<double>("map_y_max", 8.0);
  declare_parameter<double>("resolution", 0.05);
  declare_parameter<bool>("map_swap_xy", true);
  declare_parameter<bool>("map_flip_x", true);
  declare_parameter<bool>("map_flip_y", true);
  declare_parameter<double>("scale_width_mm", 16000.0);
  declare_parameter<double>("scale_height_mm", 11000.0);
  declare_parameter<int>("free_value", 0);
  declare_parameter<int>("occupied_value", 100);
  declare_parameter<int>("unknown_value", -1);
  declare_parameter<bool>("unknown_outside_drivable", false);
  declare_parameter<double>("parking_hull_margin", 0.35);
  declare_parameter<double>("parking_line_obstacle_width", 0.10);
  declare_parameter<double>("road_boundary_obstacle_width", 0.08);
  declare_parameter<bool>("mark_inner_road_boundary_obstacle", false);
  declare_parameter<bool>("mark_outer_road_boundary_obstacle", true);
  declare_parameter<std::vector<std::string>>(
    "ignored_parking_line_roles",
    std::vector<std::string>{
      "parking_lot_boundary",
      "top_slot_header",
      "bottom_slot_edge",
      "entry_direction_marking",
    });
  declare_parameter<bool>("log_map_summary", true);

  track_csv_path_ = resolveTrackCsvPath(get_parameter("track_csv_path").as_string());
  parking_csv_path_ = resolveParkingCsvPath(get_parameter("parking_csv_path").as_string());
  map_topic_ = get_parameter("map_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  publish_rate_ = std::max(0.01, get_parameter("publish_rate").as_double());
  map_x_min_ = get_parameter("map_x_min").as_double();
  map_x_max_ = get_parameter("map_x_max").as_double();
  map_y_min_ = get_parameter("map_y_min").as_double();
  map_y_max_ = get_parameter("map_y_max").as_double();
  resolution_ = std::max(0.01, get_parameter("resolution").as_double());
  map_swap_xy_ = get_parameter("map_swap_xy").as_bool();
  map_flip_x_ = get_parameter("map_flip_x").as_bool();
  map_flip_y_ = get_parameter("map_flip_y").as_bool();
  scale_width_mm_ = get_parameter("scale_width_mm").as_double();
  scale_height_mm_ = get_parameter("scale_height_mm").as_double();
  free_value_ = clampOccupancyValue(get_parameter("free_value").as_int());
  occupied_value_ = clampOccupancyValue(get_parameter("occupied_value").as_int());
  unknown_value_ = get_parameter("unknown_value").as_int() < 0 ? -1 :
    clampOccupancyValue(get_parameter("unknown_value").as_int());
  unknown_outside_drivable_ = get_parameter("unknown_outside_drivable").as_bool();
  parking_hull_margin_ = std::max(0.0, get_parameter("parking_hull_margin").as_double());
  parking_line_obstacle_width_ =
    std::max(resolution_, get_parameter("parking_line_obstacle_width").as_double());
  road_boundary_obstacle_width_ =
    std::max(0.0, get_parameter("road_boundary_obstacle_width").as_double());
  mark_inner_road_boundary_obstacle_ =
    get_parameter("mark_inner_road_boundary_obstacle").as_bool();
  mark_outer_road_boundary_obstacle_ =
    get_parameter("mark_outer_road_boundary_obstacle").as_bool();
  ignored_parking_line_roles_ =
    get_parameter("ignored_parking_line_roles").as_string_array();
  log_map_summary_ = get_parameter("log_map_summary").as_bool();

  loadInputs();
  buildMap();

  rclcpp::QoS qos(1);
  qos.transient_local().reliable();
  pub_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, qos);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / publish_rate_));
  timer_ = create_wall_timer(period_ms, std::bind(&ParkingMapPublisher::publishMap, this));

  publishMap();
  RCLCPP_INFO(
    get_logger(),
    "ParkingMapPublisher ready: topic=%s, frame=%s, size=%ux%u, resolution=%.3f, "
    "track_csv=%s, parking_csv=%s",
    map_topic_.c_str(), frame_id_.c_str(), map_.info.width, map_.info.height,
    map_.info.resolution, track_csv_path_.c_str(), parking_csv_path_.c_str());
}

std::string ParkingMapPublisher::resolveTrackCsvPath(const std::string & configured_path) const
{
  const fs::path default_dir =
    fs::path(ament_index_cpp::get_package_share_directory("parking_map_publisher")) /
    "data" / "track_line_export";
  const fs::path requested = configured_path.empty() ? default_dir : fs::path(configured_path);

  std::error_code ec;
  if (fs::is_regular_file(requested, ec)) {
    if (!isCsvFile(requested)) {
      throw std::runtime_error("track_csv_path is a file but not a CSV: " + requested.string());
    }
    return requested.string();
  }

  ec.clear();
  if (!fs::is_directory(requested, ec)) {
    throw std::runtime_error(
      "track_csv_path does not exist or is not a directory/CSV file: " + requested.string());
  }

  const fs::path preferred = requested / kTrackCsvName;
  if (fs::is_regular_file(preferred, ec)) {
    return preferred.string();
  }

  std::vector<fs::path> candidates;
  const auto options = fs::directory_options::skip_permission_denied;
  for (const auto & entry : fs::recursive_directory_iterator(requested, options)) {
    std::error_code entry_ec;
    if (entry.is_regular_file(entry_ec) && isCsvFile(entry.path())) {
      const std::string filename = lower(entry.path().filename().string());
      if (filename.find("track") != std::string::npos && filename.find("wide") != std::string::npos) {
        candidates.push_back(entry.path());
      }
    }
  }

  if (candidates.empty()) {
    throw std::runtime_error("No track wide CSV found under: " + requested.string());
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates.front().string();
}

std::string ParkingMapPublisher::resolveParkingCsvPath(const std::string & configured_path) const
{
  const fs::path default_dir =
    fs::path(ament_index_cpp::get_package_share_directory("parking_map_publisher")) /
    "data" / "track_line_export";
  const fs::path requested = configured_path.empty() ? default_dir : fs::path(configured_path);

  std::error_code ec;
  if (fs::is_regular_file(requested, ec)) {
    if (!isCsvFile(requested)) {
      throw std::runtime_error("parking_csv_path is a file but not a CSV: " + requested.string());
    }
    return requested.string();
  }

  ec.clear();
  if (!fs::is_directory(requested, ec)) {
    throw std::runtime_error(
      "parking_csv_path does not exist or is not a directory/CSV file: " + requested.string());
  }

  const fs::path preferred = requested / kParkingCsvName;
  if (fs::is_regular_file(preferred, ec)) {
    return preferred.string();
  }

  std::vector<fs::path> candidates;
  const auto options = fs::directory_options::skip_permission_denied;
  for (const auto & entry : fs::recursive_directory_iterator(requested, options)) {
    std::error_code entry_ec;
    if (entry.is_regular_file(entry_ec) && isCsvFile(entry.path())) {
      const std::string filename = lower(entry.path().filename().string());
      if (filename.find("parking") != std::string::npos) {
        candidates.push_back(entry.path());
      }
    }
  }

  if (candidates.empty()) {
    throw std::runtime_error("No parking CSV found under: " + requested.string());
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates.front().string();
}

void ParkingMapPublisher::loadInputs()
{
  road_inner_ = loadTrackColumnPair(track_csv_path_, "left_x_mm", "left_y_mm");
  road_outer_ = loadTrackColumnPair(track_csv_path_, "right_x_mm", "right_y_mm");
  parking_lines_ = loadParkingLines(parking_csv_path_);

  if (road_inner_.size() < 3 || road_outer_.size() < 3) {
    throw std::runtime_error("Track boundary CSV must contain at least 3 points per boundary.");
  }
  if (parking_lines_.empty()) {
    throw std::runtime_error("Parking line CSV did not contain any line segments.");
  }

  std::vector<Point2D> parking_points;
  parking_points.reserve(parking_lines_.size() * 2);
  for (const auto & line : parking_lines_) {
    parking_points.push_back(line.p1);
    parking_points.push_back(line.p2);
  }
  parking_hull_ = convexHull(std::move(parking_points));
  if (parking_hull_.size() < 3) {
    throw std::runtime_error("Parking line endpoints do not form a valid hull.");
  }
}

std::vector<ParkingMapPublisher::Point2D>
ParkingMapPublisher::loadTrackColumnPair(
  const std::string & csv_path,
  const std::string & x_column,
  const std::string & y_column) const
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open track CSV: " + csv_path);
  }

  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("Track CSV is empty: " + csv_path);
  }

  const auto header = splitCsvLine(line);
  const size_t col_x = findColumnIndex(header, x_column);
  const size_t col_y = findColumnIndex(header, y_column);
  const size_t max_col = std::max(col_x, col_y);

  std::vector<Point2D> points;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const auto tokens = splitCsvLine(line);
    if (tokens.size() <= max_col || tokens[col_x].empty() || tokens[col_y].empty()) {
      continue;
    }
    points.push_back(mmToWorld(std::stod(tokens[col_x]), std::stod(tokens[col_y])));
  }
  return points;
}

std::vector<ParkingMapPublisher::ParkingLine>
ParkingMapPublisher::loadParkingLines(const std::string & csv_path) const
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open parking CSV: " + csv_path);
  }

  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("Parking CSV is empty: " + csv_path);
  }

  const auto header = splitCsvLine(line);
  const size_t col_line_id = findColumnIndex(header, "line_id");
  const size_t col_role = findColumnIndex(header, "line_role");
  const size_t col_x1 = findColumnIndex(header, "x1_m");
  const size_t col_y1 = findColumnIndex(header, "y1_m");
  const size_t col_x2 = findColumnIndex(header, "x2_m");
  const size_t col_y2 = findColumnIndex(header, "y2_m");
  const size_t max_col = std::max({col_line_id, col_role, col_x1, col_y1, col_x2, col_y2});

  std::vector<ParkingLine> lines;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const auto tokens = splitCsvLine(line);
    if (tokens.size() <= max_col) {
      continue;
    }

    ParkingLine segment;
    segment.line_id = std::stoi(tokens[col_line_id]);
    segment.line_role = tokens[col_role];
    segment.p1.x = std::stod(tokens[col_x1]);
    segment.p1.y = std::stod(tokens[col_y1]);
    segment.p2.x = std::stod(tokens[col_x2]);
    segment.p2.y = std::stod(tokens[col_y2]);
    lines.push_back(segment);
  }
  return lines;
}

ParkingMapPublisher::Point2D
ParkingMapPublisher::mmToWorld(double x_mm, double y_mm) const
{
  const double u = x_mm / scale_width_mm_;
  const double v = y_mm / scale_height_mm_;

  double xn = map_swap_xy_ ? v : u;
  double yn = map_swap_xy_ ? u : v;
  if (map_flip_x_) {
    xn = 1.0 - xn;
  }
  if (map_flip_y_) {
    yn = 1.0 - yn;
  }

  return {
    map_x_min_ + xn * (map_x_max_ - map_x_min_),
    map_y_min_ + yn * (map_y_max_ - map_y_min_),
  };
}

void ParkingMapPublisher::buildMap()
{
  if (map_x_max_ <= map_x_min_ || map_y_max_ <= map_y_min_) {
    throw std::runtime_error("Invalid map bounds.");
  }

  map_.header.frame_id = frame_id_;
  map_.info.resolution = resolution_;
  map_.info.width = static_cast<uint32_t>(std::ceil((map_x_max_ - map_x_min_) / resolution_));
  map_.info.height = static_cast<uint32_t>(std::ceil((map_y_max_ - map_y_min_) / resolution_));
  map_.info.origin.position.x = map_x_min_;
  map_.info.origin.position.y = map_y_min_;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(
    static_cast<size_t>(map_.info.width) * static_cast<size_t>(map_.info.height),
    static_cast<int8_t>(unknown_outside_drivable_ ? unknown_value_ : occupied_value_));

  size_t drivable_cells = 0;
  for (uint32_t iy = 0; iy < map_.info.height; ++iy) {
    for (uint32_t ix = 0; ix < map_.info.width; ++ix) {
      const Point2D point = gridToWorld(static_cast<int>(ix), static_cast<int>(iy));
      const bool on_road = pointInPolygon(road_outer_, point) &&
        !pointInPolygon(road_inner_, point);

      bool in_parking = pointInPolygon(parking_hull_, point);
      if (!in_parking && parking_hull_margin_ > 0.0) {
        for (size_t i = 0; i < parking_hull_.size(); ++i) {
          const auto & a = parking_hull_[i];
          const auto & b = parking_hull_[(i + 1) % parking_hull_.size()];
          if (distanceToSegment(point, a, b) <= parking_hull_margin_) {
            in_parking = true;
            break;
          }
        }
      }

      if (on_road || in_parking) {
        setCell(static_cast<int>(ix), static_cast<int>(iy), free_value_);
        ++drivable_cells;
      }
    }
  }

  if (mark_outer_road_boundary_obstacle_) {
    for (size_t i = 0; i < road_outer_.size(); ++i) {
      drawLineObstacle(road_outer_[i], road_outer_[(i + 1) % road_outer_.size()],
        road_boundary_obstacle_width_);
    }
  }
  if (mark_inner_road_boundary_obstacle_) {
    for (size_t i = 0; i < road_inner_.size(); ++i) {
      drawLineObstacle(road_inner_[i], road_inner_[(i + 1) % road_inner_.size()],
        road_boundary_obstacle_width_);
    }
  }
  size_t obstacle_line_count = 0;
  size_t ignored_line_count = 0;
  for (const auto & line : parking_lines_) {
    if (isIgnoredParkingLineRole(line.line_role)) {
      ++ignored_line_count;
      continue;
    }
    drawLineObstacle(line.p1, line.p2, parking_line_obstacle_width_);
    ++obstacle_line_count;
  }
  for (const auto & line : parking_lines_) {
    if (isIgnoredParkingLineRole(line.line_role)) {
      drawLineValue(
        line.p1, line.p2,
        parking_line_obstacle_width_ + resolution_,
        free_value_);
    }
  }

  if (log_map_summary_) {
    size_t free_cells = 0;
    size_t occupied_cells = 0;
    size_t unknown_cells = 0;
    for (const auto value : map_.data) {
      if (value < 0) {
        ++unknown_cells;
      } else if (value == free_value_) {
        ++free_cells;
      } else if (value >= occupied_value_) {
        ++occupied_cells;
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "Built parking occupancy map: drivable_before_lines=%zu, free=%zu, occupied=%zu, unknown=%zu, "
      "parking_lines_obstacle=%zu, parking_lines_ignored=%zu, parking_hull_vertices=%zu",
      drivable_cells, free_cells, occupied_cells, unknown_cells,
      obstacle_line_count, ignored_line_count, parking_hull_.size());
  }
}

bool ParkingMapPublisher::isIgnoredParkingLineRole(const std::string & role) const
{
  return std::find(
    ignored_parking_line_roles_.begin(), ignored_parking_line_roles_.end(), role) !=
    ignored_parking_line_roles_.end();
}

void ParkingMapPublisher::publishMap()
{
  map_.header.stamp = now();
  map_.info.map_load_time = now();
  pub_map_->publish(map_);
}

std::vector<std::string> ParkingMapPublisher::splitCsvLine(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    tokens.push_back(trim(token));
  }
  return tokens;
}

std::string ParkingMapPublisher::trim(const std::string & value)
{
  size_t first = 0;
  while (first < value.size() && std::isspace(static_cast<unsigned char>(value[first]))) {
    ++first;
  }

  size_t last = value.size();
  while (last > first && std::isspace(static_cast<unsigned char>(value[last - 1]))) {
    --last;
  }
  return value.substr(first, last - first);
}

size_t ParkingMapPublisher::findColumnIndex(
  const std::vector<std::string> & header,
  const std::string & column_name)
{
  const auto it = std::find(header.begin(), header.end(), column_name);
  if (it == header.end()) {
    throw std::runtime_error("Missing required CSV column: " + column_name);
  }
  return static_cast<size_t>(std::distance(header.begin(), it));
}

bool ParkingMapPublisher::pointInPolygon(
  const std::vector<Point2D> & polygon,
  const Point2D & point)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    const bool intersects = ((pi.y > point.y) != (pj.y > point.y)) &&
      (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y + kEpsilon) + pi.x);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<ParkingMapPublisher::Point2D>
ParkingMapPublisher::convexHull(std::vector<Point2D> points)
{
  if (points.size() <= 1) {
    return points;
  }

  std::sort(points.begin(), points.end(), [](const Point2D & a, const Point2D & b) {
    if (std::fabs(a.x - b.x) > kEpsilon) {
      return a.x < b.x;
    }
    return a.y < b.y;
  });
  points.erase(std::unique(points.begin(), points.end(), [](const Point2D & a, const Point2D & b) {
    return std::fabs(a.x - b.x) <= kEpsilon && std::fabs(a.y - b.y) <= kEpsilon;
  }), points.end());

  std::vector<Point2D> hull;
  hull.reserve(points.size() * 2);

  for (const auto & point : points) {
    while (hull.size() >= 2 &&
      cross(hull[hull.size() - 2], hull.back(), point) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }

  const size_t lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower_size &&
      cross(hull[hull.size() - 2], hull.back(), *it) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(*it);
  }

  if (!hull.empty()) {
    hull.pop_back();
  }
  return hull;
}

double ParkingMapPublisher::cross(const Point2D & origin, const Point2D & a, const Point2D & b)
{
  return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

double ParkingMapPublisher::distanceToSegment(
  const Point2D & point,
  const Point2D & a,
  const Point2D & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = point.x - a.x;
  const double wy = point.y - a.y;
  const double len2 = vx * vx + vy * vy;
  if (len2 <= kEpsilon) {
    return std::hypot(point.x - a.x, point.y - a.y);
  }
  const double t = std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / len2));
  const double px = a.x + t * vx;
  const double py = a.y + t * vy;
  return std::hypot(point.x - px, point.y - py);
}

bool ParkingMapPublisher::worldToGrid(double x, double y, int & ix, int & iy) const
{
  if (x < map_x_min_ || y < map_y_min_ || x >= map_x_max_ || y >= map_y_max_) {
    return false;
  }
  ix = static_cast<int>(std::floor((x - map_x_min_) / resolution_));
  iy = static_cast<int>(std::floor((y - map_y_min_) / resolution_));
  return ix >= 0 && iy >= 0 &&
    ix < static_cast<int>(map_.info.width) && iy < static_cast<int>(map_.info.height);
}

ParkingMapPublisher::Point2D ParkingMapPublisher::gridToWorld(int ix, int iy) const
{
  return {
    map_x_min_ + (static_cast<double>(ix) + 0.5) * resolution_,
    map_y_min_ + (static_cast<double>(iy) + 0.5) * resolution_,
  };
}

void ParkingMapPublisher::setCell(int ix, int iy, int value)
{
  if (ix < 0 || iy < 0 ||
    ix >= static_cast<int>(map_.info.width) || iy >= static_cast<int>(map_.info.height))
  {
    return;
  }
  const auto index = static_cast<size_t>(iy) * map_.info.width + static_cast<size_t>(ix);
  if (index < map_.data.size()) {
    map_.data[index] = static_cast<int8_t>(value);
  }
}

void ParkingMapPublisher::drawLineObstacle(const Point2D & a, const Point2D & b, double width_m)
{
  drawLineValue(a, b, width_m, occupied_value_);
}

void ParkingMapPublisher::drawLineValue(
  const Point2D & a,
  const Point2D & b,
  double width_m,
  int value)
{
  if (width_m <= 0.0) {
    return;
  }

  const double radius = width_m * 0.5;
  const double min_x = std::min(a.x, b.x) - radius;
  const double max_x = std::max(a.x, b.x) + radius;
  const double min_y = std::min(a.y, b.y) - radius;
  const double max_y = std::max(a.y, b.y) + radius;

  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const int ix0 = std::max(0, static_cast<int>(std::floor((min_x - map_x_min_) / resolution_)));
  const int iy0 = std::max(0, static_cast<int>(std::floor((min_y - map_y_min_) / resolution_)));
  const int ix1 = std::min(width - 1, static_cast<int>(std::ceil((max_x - map_x_min_) / resolution_)));
  const int iy1 = std::min(height - 1, static_cast<int>(std::ceil((max_y - map_y_min_) / resolution_)));

  if (ix1 < ix0 || iy1 < iy0) {
    return;
  }

  for (int iy = iy0; iy <= iy1; ++iy) {
    for (int ix = ix0; ix <= ix1; ++ix) {
      const Point2D point = gridToWorld(ix, iy);
      if (distanceToSegment(point, a, b) <= radius) {
        setCell(ix, iy, value);
      }
    }
  }
}
