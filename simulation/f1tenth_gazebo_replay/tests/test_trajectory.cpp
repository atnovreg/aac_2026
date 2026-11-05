#include "Trajectory.hh"
#include <cassert>
#include <cmath>
#include <sstream>

int main()
{
  std::istringstream csv("t,x,y,yaw,vx\n0,1,2,3.10,1\n2,5,6,-3.10,1\n");
  replay::Trajectory trajectory(csv);
  auto sample = trajectory.At(1.0);
  assert(std::abs(sample.x-3.) < 1e-12);
  assert(std::abs(sample.y-4.) < 1e-12);
  assert(std::abs(sample.yaw-M_PI) < 1e-12);
  assert(trajectory.At(-1).x == 1.);
  assert(trajectory.At(3).x == 5.);
  for (const char *bad : {
      "t,x,y,yaw\n0,0,0,0\n0,1,1,0\n",
      "t,x,y,yaw\n0,0,0,0\n1,nan,1,0\n"})
  {
    bool rejected = false;
    try { std::istringstream input(bad); replay::Trajectory unused(input); }
    catch (const std::exception &) { rejected = true; }
    assert(rejected);
  }
}
