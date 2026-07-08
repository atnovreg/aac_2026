#include "parking_lines_publisher/parking_lines_publisher.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace
{
std::string trim(const std::string & value)
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

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}
}  // namespace

ParkingLinesPublisher::ParkingLinesPublisher(const std::string & node_name)
: Node(node_name)
{
  declare_parameter<std::string>("csv_path", "");
  declare_parameter<std::string>("marker_topic", "/parking_lines/markers");
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<double>("publish_rate", 1.0);
  declare_parameter<double>("line_width", 0.06);
  declare_parameter<double>("z_offset", 0.08);
  declare_parameter<bool>("publish_labels", false);

  csv_path_ = resolveCsvPath(get_parameter("csv_path").as_string());
  marker_topic_ = get_parameter("marker_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  publish_rate_ = std::max(0.01, get_parameter("publish_rate").as_double());
  line_width_ = std::max(0.005, get_parameter("line_width").as_double());
  z_offset_ = get_parameter("z_offset").as_double();
  publish_labels_ = get_parameter("publish_labels").as_bool();

  loadCsv();

  rclcpp::QoS qos(1);
  qos.transient_local().reliable();
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, qos);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / publish_rate_));
  timer_ = create_wall_timer(period_ms, std::bind(&ParkingLinesPublisher::timerCallback, this));

  timerCallback();

  RCLCPP_INFO(
    get_logger(),
    "ParkingLinesPublisher ready: %zu segments, topic=%s, frame=%s, rate=%.2f Hz, csv=%s",
    lines_.size(), marker_topic_.c_str(), frame_id_.c_str(), publish_rate_, csv_path_.c_str());
}

std::string ParkingLinesPublisher::resolveCsvPath(const std::string & configured_path) const
{
  const fs::path default_csv =
    fs::path(ament_index_cpp::get_package_share_directory("parking_lines_publisher")) /
    "data" / "track_line_export" / "parking_lot_lines.csv";
  const fs::path requested = configured_path.empty() ? default_csv : fs::path(configured_path);

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

  const fs::path preferred = requested / "parking_lot_lines.csv";
  if (fs::is_regular_file(preferred, ec)) {
    return preferred.string();
  }

  std::vector<fs::path> csv_files;
  const auto options = fs::directory_options::skip_permission_denied;
  for (const auto & entry : fs::recursive_directory_iterator(requested, options)) {
    std::error_code entry_ec;
    if (entry.is_regular_file(entry_ec) && isCsvFile(entry.path())) {
      const std::string filename = lower(entry.path().filename().string());
      if (filename.find("parking") != std::string::npos) {
        csv_files.push_back(entry.path());
      }
    }
  }

  if (csv_files.empty()) {
    throw std::runtime_error(
      "No parking CSV files found under csv_path directory: " + requested.string());
  }

  std::sort(csv_files.begin(), csv_files.end());
  RCLCPP_INFO(
    get_logger(), "Resolved csv_path directory %s to CSV %s",
    requested.string().c_str(), csv_files.front().string().c_str());
  return csv_files.front().string();
}

void ParkingLinesPublisher::loadCsv()
{
  std::ifstream file(csv_path_);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open CSV: " + csv_path_);
  }

  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("CSV is empty: " + csv_path_);
  }

  const auto header = splitCsvLine(line);
  const size_t col_line_id = findColumnIndex(header, "line_id");
  const size_t col_line_type = findColumnIndex(header, "line_type");
  const size_t col_line_role = findColumnIndex(header, "line_role");
  const size_t col_orientation = findColumnIndex(header, "orientation");
  const size_t col_x1 = findColumnIndex(header, "x1_m");
  const size_t col_y1 = findColumnIndex(header, "y1_m");
  const size_t col_x2 = findColumnIndex(header, "x2_m");
  const size_t col_y2 = findColumnIndex(header, "y2_m");
  const size_t col_length = findColumnIndex(header, "length_m");
  const size_t max_col = std::max({
    col_line_id, col_line_type, col_line_role, col_orientation,
    col_x1, col_y1, col_x2, col_y2, col_length});

  lines_.clear();
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    const auto tokens = splitCsvLine(line);
    if (tokens.size() <= max_col) {
      continue;
    }

    LineSegment segment;
    segment.line_id = std::stoi(tokens[col_line_id]);
    segment.line_type = tokens[col_line_type];
    segment.line_role = tokens[col_line_role];
    segment.orientation = tokens[col_orientation];
    segment.x1 = std::stod(tokens[col_x1]);
    segment.y1 = std::stod(tokens[col_y1]);
    segment.x2 = std::stod(tokens[col_x2]);
    segment.y2 = std::stod(tokens[col_y2]);
    segment.length = std::stod(tokens[col_length]);
    lines_.push_back(segment);
  }

  if (lines_.empty()) {
    throw std::runtime_error("No parking line rows loaded from CSV: " + csv_path_);
  }
}

ParkingLinesPublisher::Color ParkingLinesPublisher::colorForRole(const std::string & role) const
{
  if (role == "parking_lot_boundary") {
    return {1.0f, 0.20f, 0.18f, 0.95f};
  }
  if (role == "top_slot_header" || role == "bottom_slot_edge") {
    return {1.0f, 0.88f, 0.05f, 0.95f};
  }
  if (role == "top_slot_separator") {
    return {0.10f, 0.65f, 1.0f, 0.95f};
  }
  if (role == "bottom_slot_separator") {
    return {0.05f, 1.0f, 0.45f, 0.95f};
  }
  if (role == "entry_direction_marking") {
    return {1.0f, 0.50f, 0.05f, 0.95f};
  }
  if (role == "right_approach_marking") {
    return {0.85f, 0.35f, 1.0f, 0.95f};
  }
  return {1.0f, 1.0f, 1.0f, 0.95f};
}

std::string ParkingLinesPublisher::namespaceForRole(const std::string & role) const
{
  return "parking_lines_" + role;
}

visualization_msgs::msg::MarkerArray ParkingLinesPublisher::buildMarkers() const
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  const auto stamp = now();
  std::map<std::string, visualization_msgs::msg::Marker> role_markers;

  for (const auto & line : lines_) {
    auto & marker = role_markers[line.line_role];
    if (marker.points.empty()) {
      marker.header.frame_id = frame_id_;
      marker.header.stamp = stamp;
      marker.ns = namespaceForRole(line.line_role);
      marker.id = static_cast<int>(role_markers.size());
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = line_width_;
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 0;
      const auto color = colorForRole(line.line_role);
      marker.color.r = color.r;
      marker.color.g = color.g;
      marker.color.b = color.b;
      marker.color.a = color.a;
    }

    marker.points.push_back(makePoint(line.x1, line.y1, z_offset_));
    marker.points.push_back(makePoint(line.x2, line.y2, z_offset_));
  }

  for (const auto & [_, marker] : role_markers) {
    markers.markers.push_back(marker);
  }

  if (publish_labels_) {
    int id = 1000;
    for (const auto & line : lines_) {
      visualization_msgs::msg::Marker label;
      label.header.frame_id = frame_id_;
      label.header.stamp = stamp;
      label.ns = "parking_lines_labels";
      label.id = id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = 0.5 * (line.x1 + line.x2);
      label.pose.position.y = 0.5 * (line.y1 + line.y2);
      label.pose.position.z = z_offset_ + 0.12;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.22;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 0.95f;
      label.text = std::to_string(line.line_id) + ":" + line.line_role;
      markers.markers.push_back(label);
    }
  }

  return markers;
}

void ParkingLinesPublisher::timerCallback()
{
  pub_markers_->publish(buildMarkers());
}
