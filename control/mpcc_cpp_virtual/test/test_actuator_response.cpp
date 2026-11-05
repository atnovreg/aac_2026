#include "mpcc_cpp_virtual/actuator_response.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using mpcc_cpp_virtual::ActuatorParameters;
using mpcc_cpp_virtual::ActuatorResponse;

void near(double actual, double expected) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-12)
    throw std::runtime_error("actuator response differs from analytical fixture");
}

int main() {
  try {
    // Removing the zero-delay bypass would change the existing virtual plant.
    ActuatorResponse direct({}, .01);
    auto applied = direct.step({.7, -.2, 3.});
    near(applied[0], .7); near(applied[1], -.2); near(applied[2], 3.);

    // Off-by-one delay history must not deliver a command one tick early.
    ActuatorParameters delay;
    delay.drive_delay_s = .02; delay.steering_delay_s = .01;
    ActuatorResponse delayed(delay, .01);
    applied = delayed.step({1., .2, 2.});
    near(applied[0], 0.); near(applied[1], 0.); near(applied[2], 2.);
    applied = delayed.step({.4, .3, 3.});
    near(applied[0], 0.); near(applied[1], .2);
    applied = delayed.step({-.5, .4, 4.});
    near(applied[0], 1.); near(applied[1], .3);
    applied = delayed.step({0., 0., 0.});
    near(applied[0], .4); near(applied[1], .4);
    applied = delayed.step({0., 0., 0.});
    near(applied[0], -.5); near(applied[1], 0.);

    // The applied input is the interval mean, not the end-of-step value.
    // Fixtures are the analytical unit step at dt=.01, tau=.1.
    ActuatorParameters lag;
    lag.drive_tau_s = .1; lag.steering_tau_s = .1;
    ActuatorResponse response(lag, .01);
    applied = response.step({1., .2, 5.});
    near(applied[0], .04837418035959573);
    near(applied[1], .009674836071919147);
    near(response.endpoint()[0], .09516258196404043);
    for (int i = 1; i < 10; ++i) response.step({1., .2, 5.});
    near(response.endpoint()[0], .6321205588285577);
    // Very slow finite responses must not create nonfinite plant inputs.
    lag.drive_tau_s = std::numeric_limits<double>::max();
    ActuatorResponse slow(lag, .01);
    applied = slow.step({1., 0., 0.});
    near(applied[0], 0.);
    near(response.endpoint()[1], .12642411176571154);

    // A failed/unpermitted virtual tick clears pending commands on resume.
    delayed.reset();
    applied = delayed.step({0., 0., 1.});
    near(applied[0], 0.); near(applied[1], 0.);
    applied = delayed.step({0., 0., 1.});
    near(applied[0], 0.); near(applied[1], 0.);

    // Fractional dead times are rejected instead of silently mislabelled.
    int rejected = 0;
    for (double value : {-.01, .015, std::numeric_limits<double>::quiet_NaN()}) {
      delay.drive_delay_s = value;
      try { ActuatorResponse invalid(delay, .01); }
      catch (const std::invalid_argument&) { ++rejected; }
    }
    if (rejected != 3) return 6;
    rejected = 0;
    try { response.step({std::numeric_limits<double>::quiet_NaN(), 0., 0.}); }
    catch (const std::invalid_argument&) { ++rejected; }
    if (rejected != 1) return 7;
    near(response.endpoint()[0], .6321205588285577);
    // Prediction needs oldest-first pending commands, not only the last request.
    ActuatorParameters queue_parameters;
    queue_parameters.drive_delay_s = .02;
    queue_parameters.steering_delay_s = .04;
    ActuatorResponse queued(queue_parameters, .01);
    queued.step({.1, -.1, 1.});
    queued.step({.2, -.2, 2.});
    const auto memory = queued.prediction_state<8>();
    const std::array<double, 8> expected{0., 0., .1, .2, 0., 0., -.1, -.2};
    for (int j = 0; j < 8; ++j) near(memory[j], expected[j]);
    std::cout << "actuator response: bypass, dead time, mean lag, reset, validation PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
