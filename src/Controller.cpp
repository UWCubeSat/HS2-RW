#include "Controller.hpp"

namespace controller {
float PID::Compute(const float setpoint, const float curr_pos, const float dt) {
  float error = CalcError(setpoint, curr_pos);
  
  float output;
  output = kP*error + kI*accumulated_err_ + (dt == 0) ? 0 : kD*(error - prev_err_)/dt;
  
  prev_err_ = error;
  accumulated_err_ += error*dt;

  return output;
}

void PID::Reset() {
  prev_err_ = 0;
  accumulated_err_ = 0;
}

float PID::CalcError(const float setpoint, const float curr_pos) {
  return setpoint - curr_pos;
}
}  // namespace controller
