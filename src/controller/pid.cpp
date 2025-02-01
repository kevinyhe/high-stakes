#include "controller/pid.hpp"
#include "math/math.hpp"

double PID::calculate(double current, double target) {
  return calculate_error(target - current);
}

double PID::calculate_error(double error) {
  if (last_error == INFINITY) {
    last_error = error;
  }

  if (reset_integral_on_cross && sgn(error) != sgn(last_error)) {
    total_error = 0;
  }

  if (error < integral_active_zone) {
    total_error += error;
  } else {
    total_error = 0;
  }

  if (total_error > integral_max) {
    total_error = integral_max;
  }

  if (error < integral_reset_zone) {
    total_error = 0;
  }

  double proportional = error * kP;
  double integral = total_error * kI;
  double derivative = (error - last_error) * kD;

  last_error = error;

  double output = proportional + integral + derivative;

  if (std::fabs(output - last_output) > slew_rate) {
    output = last_output + slew_rate * sgn(output - last_output);
  }

  last_output = output;

  return output;
}

void PID::reset_pid() {
  total_error = 0;
  last_error = INFINITY;
}