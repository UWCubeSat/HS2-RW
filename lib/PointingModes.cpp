#include "PointingModes.hpp"
#include "Controller.hpp"
#include "RwStatus.hpp"

namespace pointing_modes {
FourWheelMode::FourWheelMode() : wheel_pwm_{0, 0, 0, 0} {
  // set the values for the pseudoinverse, as described in the doc
  pseudoinverse_.cell(0,0) = kSqrt3Div4;
  pseudoinverse_.cell(1,0) = -kSqrt3Div4;
  pseudoinverse_.cell(2,0) = -kSqrt3Div4;
  pseudoinverse_.cell(3,0) = kSqrt3Div4;

  pseudoinverse_.cell(0,1) = -kSqrt3Div4;
  pseudoinverse_.cell(1,1) = kSqrt3Div4;
  pseudoinverse_.cell(2,1) = -kSqrt3Div4;
  pseudoinverse_.cell(3,1) = kSqrt3Div4;

  pseudoinverse_.cell(0,2) = kSqrt3Div4;
  pseudoinverse_.cell(1,2) = kSqrt3Div4;
  pseudoinverse_.cell(2,2) = kSqrt3Div4;
  pseudoinverse_.cell(3,2) = kSqrt3Div4;

  pseudoinverse_.cell(0,3) = 0;
  pseudoinverse_.cell(1,3) = 0;
  pseudoinverse_.cell(2,3) = 0;
  pseudoinverse_.cell(3,3) = 0;
}

void FourWheelMode::Calculate(const imu::Vector<3>& sat_torque, float wheel_torques[]) const {
  imu::Vector<4> v(sat_torque[0], sat_torque[1], sat_torque[2], 0);
  for (uint8_t i = 0; i < kNumWheels; i++) {
    wheel_torques[i] = pseudoinverse_.row_to_vector(i).dot(v);
  }
}

void FourWheelMode::Pid_Speed(const float wheel_torques[], const uint32_t dt,
  controller::WheelSpeedPD& wpd, uint8_t pwm[]) {
  for (uint8_t i = 0; i < kNumWheels; i++) {
    // integrate (torques * wheelmoment) to get speed
    wheel_pwm_[i] += wpd.Compute(wheel_torques[i] * kWheelMoment[i] * dt, interrupt::wheel_rpm[i], dt);
    pwm[i] = wheel_pwm_[i];
  }
}
}  // namespace pointing_modes