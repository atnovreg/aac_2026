#pragma once
#include <algorithm>
#include <cmath>
#include <istream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace replay
{
struct Sample { double t, x, y, yaw; };
class Trajectory
{
  public: explicit Trajectory(std::istream &input)
  {
    std::string line;
    if (!std::getline(input, line)) throw std::runtime_error("Empty trajectory CSV");
    auto header = Split(line);
    std::vector<std::size_t> indices;
    for (const auto &field : {"t", "x", "y", "yaw"})
    {
      auto found = std::find(header.begin(), header.end(), field);
      if (found == header.end()) throw std::runtime_error("Missing trajectory field");
      indices.push_back(found-header.begin());
    }
    while (std::getline(input, line))
    {
      if (line.empty()) continue;
      auto fields = Split(line);
      std::vector<double> values;
      for (auto index : indices)
      {
        if (index >= fields.size()) throw std::runtime_error("Incomplete trajectory row");
        double value = std::stod(fields[index]);
        if (!std::isfinite(value)) throw std::runtime_error("Nonfinite trajectory value");
        values.push_back(value);
      }
      Sample sample{values[0], values[1], values[2], values[3]};
      if (!samples.empty())
      {
        if (sample.t <= samples.back().t) throw std::runtime_error("Non-increasing trajectory time");
        sample.yaw = samples.back().yaw + std::remainder(sample.yaw-samples.back().yaw, 2*M_PI);
      }
      samples.push_back(sample);
    }
    if (samples.size() < 2) throw std::runtime_error("Need two trajectory samples");
  }
  public: double Duration() const { return samples.back().t-samples.front().t; }
  public: Sample At(double elapsed) const
  {
    double t = elapsed + samples.front().t;
    if (t <= samples.front().t) return samples.front();
    if (t >= samples.back().t) return samples.back();
    auto next = std::upper_bound(samples.begin(), samples.end(), t,
      [](double value, const Sample &sample) { return value < sample.t; });
    const auto &previous = *(next-1);
    double a = (t-previous.t)/(next->t-previous.t);
    return {t, previous.x+a*(next->x-previous.x), previous.y+a*(next->y-previous.y),
            previous.yaw+a*(next->yaw-previous.yaw)};
  }
  private: static std::vector<std::string> Split(const std::string &line)
  {
    std::istringstream input(line);
    std::vector<std::string> result;
    std::string value;
    while (std::getline(input, value, ',')) result.push_back(value);
    return result;
  }
  private: std::vector<Sample> samples;
};
}
