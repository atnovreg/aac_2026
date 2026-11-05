#include "mpcc_cpp_virtual/core.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  const auto track = mpcc_cpp_virtual::Track::load(argv[1]);
  const auto params = mpcc_cpp_virtual::RuntimeParameters::load(argv[2]);
  mpcc_cpp_virtual::Core a(track, params), b(track, params);
  bool rejected = false;
  try { a.write_diagnostics("/tmp/mpcc_test_before_solve"); }
  catch (const std::logic_error&) { rejected = true; }
  if (!rejected) return 3;
  const auto r = track.reference(0);
  std::array<double, 7> state{r.center[0], r.center[1],
      std::atan2(r.tangent[1], r.tangent[0]), params.initial_speed, 0, 0, 0};
  mpcc_cpp_virtual::ActuatorResponse actor(params.prediction_actuator, .01);
  const auto first = a.step(state, {}, .01, &actor);
  const auto other = b.step(state, {}, .01, &actor);
  if (first.status || other.status) return 4;
  const std::string prefix = "/tmp/mpcc_test_diagnostics";
  a.write_diagnostics(prefix);
  std::ifstream snapshot(prefix + "_snapshot.json"), qp(prefix + "_qp.json");
  std::stringstream s, q; s << snapshot.rdbuf(); q << qp.rdbuf();
  if (s.str().find("\"x_guess\"") == std::string::npos ||
      s.str().find("\"statistics_rows\"") == std::string::npos ||
      s.str().find("\"nx\": " + std::to_string(mpcc_cpp_virtual::kStateDimension)) == std::string::npos ||
      q.str().find("A_00") == std::string::npos) return 5;
  const auto next = a.step(state, first.command, .01, &actor);
  const auto expected = b.step(state, other.command, .01, &actor);
  if (next.status != expected.status) return 6;
  for (int i = 0; i < 3; ++i)
    if (std::abs(next.command[i] - expected.command[i]) > 1e-10) return 7;
  auto invalid = state;
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  if (a.step(invalid, {}, .01, &actor).status != -2) return 8;
  rejected = false;
  try { a.write_diagnostics(prefix); }
  catch (const std::logic_error&) { rejected = true; }
  if (!rejected) return 9;  // Never label a previous QP as the current failure.
  std::remove((prefix + "_snapshot.json").c_str());
  std::remove((prefix + "_qp.json").c_str());
  return 0;
}
