#include "track_path_publisher/track_path_publisher.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace
{
std::string trim(const std::string & value)
{
  size_t first = 0;
  while (first < value.size() &&
    std::isspace(static_cast<unsigned char>(value[first])))
  {
    ++first;
  }

  size_t last = value.size();
  while (last > first &&
    std::isspace(static_cast<unsigned char>(value[last - 1])))
  {
    --last;
  }

  return value.substr(first, last - first);
}

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    tokens.push_back(trim(token));
  }
  return tokens;
}

size_t findColumnIndex(
  const std::vector<std::string> & header,
  const std::string & column_name)
{
  const auto it = std::find(header.begin(), header.end(), column_name);
  if (it == header.end()) {
    throw std::runtime_error("Missing required CSV column: " + column_name);
  }
  return static_cast<size_t>(std::distance(header.begin(), it));
}

bool isCsvFile(const fs::path & path)
{
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return ext == ".csv";
}

bool isWideCsvFile(const fs::path & path)
{
  std::string filename = path.filename().string();
  std::transform(filename.begin(), filename.end(), filename.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return filename.find("_wide.csv") != std::string::npos;
}
}  // namespace

static constexpr const char * COL_LEFT_X_MM = "left_x_mm";
static constexpr const char * COL_LEFT_Y_MM = "left_y_mm";
static constexpr const char * COL_CENTER_X_MM = "center_x_mm";
static constexpr const char * COL_CENTER_Y_MM = "center_y_mm";
static constexpr const char * COL_RIGHT_X_MM = "right_x_mm";
static constexpr const char * COL_RIGHT_Y_MM = "right_y_mm";

TrackPathPublisher::TrackPathPublisher(const std::string & node_name)
: Node(node_name)
{
  declare_parameter<std::string>("csv_path", "");
  declare_parameter<double>("waypoint_sampling_dist", 0.2);
  declare_parameter<double>("publish_rate", 1.0);
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<double>("map_x_min", -5.5);
  declare_parameter<double>("map_x_max",  5.5);
  declare_parameter<double>("map_y_min", -8.0);
  declare_parameter<double>("map_y_max",  8.0);
  declare_parameter<bool>("map_swap_xy", true);
  declare_parameter<bool>("map_flip_x",  true);
  declare_parameter<bool>("map_flip_y",  false);
  declare_parameter<double>("scale_width_mm",  16000.0);
  declare_parameter<double>("scale_height_mm", 11000.0);

  csv_path_               = resolveCsvPath(get_parameter("csv_path").as_string());
  waypoint_sampling_dist_ = get_parameter("waypoint_sampling_dist").as_double();
  publish_rate_           = get_parameter("publish_rate").as_double();
  frame_id_               = get_parameter("frame_id").as_string();
  map_x_min_              = get_parameter("map_x_min").as_double();
  map_x_max_              = get_parameter("map_x_max").as_double();
  map_y_min_              = get_parameter("map_y_min").as_double();
  map_y_max_              = get_parameter("map_y_max").as_double();
  map_swap_xy_            = get_parameter("map_swap_xy").as_bool();
  map_flip_x_             = get_parameter("map_flip_x").as_bool();
  map_flip_y_             = get_parameter("map_flip_y").as_bool();
  scale_width_mm_         = get_parameter("scale_width_mm").as_double();
  scale_height_mm_        = get_parameter("scale_height_mm").as_double();

  // Latched QoS: late subscribers receive the last published message
  rclcpp::QoS qos(1);
  qos.transient_local();

  pub_left_   = create_publisher<nav_msgs::msg::Path>("/path/left",   qos);
  pub_center_ = create_publisher<nav_msgs::msg::Path>("/path/center", qos);
  pub_right_  = create_publisher<nav_msgs::msg::Path>("/path/right",  qos);

  // Build paths once at startup
  loadAndPublish();

  // Publish periodically so late subscribers always get fresh data
  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / std::max(publish_rate_, 0.01)));
  timer_ = create_wall_timer(period_ms, std::bind(&TrackPathPublisher::timerCallback, this));
}

std::string TrackPathPublisher::resolveCsvPath(const std::string & configured_path) const
{
  const fs::path default_dir =
    fs::path(ament_index_cpp::get_package_share_directory("track_path_publisher")) /
    "data" / "track_line_export";
  const fs::path requested = configured_path.empty() ? default_dir : fs::path(configured_path);

  std::error_code ec;
  if (fs::is_regular_file(requested, ec)) {
    if (!isCsvFile(requested)) {
      throw std::runtime_error("csv_path is a file but not a CSV: " + requested.string());
    }
    return requested.string();
  }

  ec.clear();
  if (!fs::is_directory(requested, ec)) {
    throw std::runtime_error(
      "csv_path does not exist or is not a directory/CSV file: " + requested.string());
  }

  std::vector<fs::path> csv_files;
  const auto options = fs::directory_options::skip_permission_denied;
  for (const auto & entry : fs::recursive_directory_iterator(requested, options)) {
    std::error_code entry_ec;
    if (entry.is_regular_file(entry_ec) && isCsvFile(entry.path())) {
      csv_files.push_back(entry.path());
    }
  }

  if (csv_files.empty()) {
    throw std::runtime_error("No CSV files found under csv_path directory: " + requested.string());
  }

  std::sort(csv_files.begin(), csv_files.end());
  std::vector<fs::path> wide_csv_files;
  for (const auto & path : csv_files) {
    if (isWideCsvFile(path)) {
      wide_csv_files.push_back(path);
    }
  }

  const auto & candidates = wide_csv_files.empty() ? csv_files : wide_csv_files;
  if (candidates.size() > 1) {
    RCLCPP_WARN(
      get_logger(),
      "Multiple CSV files found under %s; using %s",
      requested.string().c_str(), candidates.front().string().c_str());
  }

  RCLCPP_INFO(
    get_logger(),
    "Resolved csv_path directory %s to CSV %s",
    requested.string().c_str(), candidates.front().string().c_str());

  return candidates.front().string();
}

void TrackPathPublisher::loadAndPublish()
{
  const auto left_pts   = resample(
    parseCsvColumns(csv_path_, COL_LEFT_X_MM,   COL_LEFT_Y_MM),
    waypoint_sampling_dist_);
  const auto center_pts = resample(
    parseCsvColumns(csv_path_, COL_CENTER_X_MM, COL_CENTER_Y_MM),
    waypoint_sampling_dist_);
  const auto right_pts  = resample(
    parseCsvColumns(csv_path_, COL_RIGHT_X_MM,  COL_RIGHT_Y_MM),
    waypoint_sampling_dist_);

  path_left_   = toPathMsg(left_pts);
  path_center_ = toPathMsg(center_pts);
  path_right_  = toPathMsg(right_pts);

  RCLCPP_INFO(
    get_logger(),
    "Loaded paths — left: %zu  center: %zu  right: %zu waypoints "
    "(sampling_dist=%.3f m, rate=%.1f Hz, csv=%s)",
    left_pts.size(), center_pts.size(), right_pts.size(),
    waypoint_sampling_dist_, publish_rate_, csv_path_.c_str());
}

void TrackPathPublisher::timerCallback()
{
  const auto stamp = now();
  path_left_.header.stamp   = stamp;
  path_center_.header.stamp = stamp;
  path_right_.header.stamp  = stamp;

  pub_left_->publish(path_left_);
  pub_center_->publish(path_center_);
  pub_right_->publish(path_right_);
}

std::vector<TrackPathPublisher::Pt>
TrackPathPublisher::parseCsvColumns(
  const std::string & csv_path,
  const std::string & col_x_mm,
  const std::string & col_y_mm) const
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open CSV: " + csv_path);
  }

  std::vector<Pt> pts;
  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("CSV is empty: " + csv_path);
  }

  const auto header = splitCsvLine(line);
  const size_t col_x_index = findColumnIndex(header, col_x_mm);
  const size_t col_y_index = findColumnIndex(header, col_y_mm);
  const size_t max_col = std::max(col_x_index, col_y_index);

  while (std::getline(file, line)) {
    if (line.empty()) { continue; }

    const auto tokens = splitCsvLine(line);

    if (tokens.size() <= max_col) { continue; }
    if (tokens[col_x_index].empty() || tokens[col_y_index].empty()) { continue; }

    const double x_mm = std::stod(tokens[col_x_index]);
    const double y_mm = std::stod(tokens[col_y_index]);
    pts.push_back(mmToWorld(x_mm, y_mm));
  }

  return pts;
}

TrackPathPublisher::Pt
TrackPathPublisher::mmToWorld(double x_mm, double y_mm) const
{
  // Normalise to [0, 1] in image UV space
  const double u = x_mm / scale_width_mm_;
  const double v = y_mm / scale_height_mm_;

  // Apply the same axis swap/flip as image_match_node
  double xn = map_swap_xy_ ? v : u;
  double yn = map_swap_xy_ ? u : v;
  if (map_flip_x_) { xn = 1.0 - xn; }
  if (map_flip_y_) { yn = 1.0 - yn; }

  return {
    map_x_min_ + xn * (map_x_max_ - map_x_min_),
    map_y_min_ + yn * (map_y_max_ - map_y_min_),
  };
}

std::vector<TrackPathPublisher::Pt>
TrackPathPublisher::resample(
  const std::vector<Pt> & pts, double spacing_m) const
{
  if (pts.size() < 2 || spacing_m <= 0.0) { return pts; }

  // Cumulative arc-length at each original point
  std::vector<double> arc(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    const double dx = pts[i].x - pts[i - 1].x;
    const double dy = pts[i].y - pts[i - 1].y;
    arc[i] = arc[i - 1] + std::sqrt(dx * dx + dy * dy);
  }

  std::vector<Pt> result;
  size_t seg = 0;
  for (double s = 0.0; s <= arc.back(); s += spacing_m) {
    while (seg + 1 < pts.size() - 1 && arc[seg + 1] <= s) { ++seg; }

    const double seg_len = arc[seg + 1] - arc[seg];
    double t = (seg_len > 1e-9) ? (s - arc[seg]) / seg_len : 0.0;
    t = std::max(0.0, std::min(1.0, t));

    result.push_back({
      pts[seg].x + t * (pts[seg + 1].x - pts[seg].x),
      pts[seg].y + t * (pts[seg + 1].y - pts[seg].y),
    });
  }

  return result;
}

nav_msgs::msg::Path
TrackPathPublisher::toPathMsg(const std::vector<Pt> & pts) const
{
  nav_msgs::msg::Path path;
  path.header.stamp    = now();
  path.header.frame_id = frame_id_;

  for (size_t i = 0; i < pts.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = pts[i].x;
    ps.pose.position.y = pts[i].y;
    ps.pose.position.z = 0.0;

    // Heading: direction toward the next waypoint
    double yaw = 0.0;
    if (i + 1 < pts.size()) {
      yaw = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
    } else if (i > 0) {
      yaw = std::atan2(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
    }

    // Quaternion from yaw (roll = pitch = 0)
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = std::sin(yaw * 0.5);
    ps.pose.orientation.w = std::cos(yaw * 0.5);

    path.poses.push_back(ps);
  }

  return path;
}
