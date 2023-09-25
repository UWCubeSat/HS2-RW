#ifndef RW_SRC_POINTINGMODES_HPP_
#define RW_SRC_POINTINGMODES_HPP_

#include <stdint.h>
#include <utility/imumaths.hpp>

namespace pointing_modes {

class PointingMode {
 public:
  PointingMode() = default;
  virtual ~PointingMode() = default;
  PointingMode(const PointingMode& other) = default;
  PointingMode& operator=(const PointingMode& other) = default;

  // Given a vector for a requested satellite torque, convert it to a vector
  // of wheel torques and return it via return paramter in column vector form.
  // The return parameter must be an array of floats with enough indices to
  // match each wheel to a single index.
  virtual void Calculate(const imu::Vector<3>& sat_torque,
    float* const* const wheel_torques) const = 0;

  // Given an array of requested wheel torques, PID each motor to that speed by
  // changing and returning the PWM output parameter.
  virtual void Pid_Speed(const float* const * const wheel_torques, const uint32_t dt,
    controller::WheelSpeedPD wpd, uint8_t* const* const pwm) = 0;
};

// A pointing mode which implements functions for a four wheel system
class FourWheelMode : public PointingMode {
 public:
  FourWheelMode();
  virtual ~FourWheelMode() = default;
  FourWheelMode(const FourWheelMode& other) = delete;
  FourWheelMode& operator=(const FourWheelMode& other) = delete;

  // Given a vector for a requested satellite torque, convert it to 4 wheel torques
  // by multiplying the pseudoinverse by our the torque requirement: T_w = Z+ * T_s
  // and return it in column vector form.
  // TODO T_w = Z+ * (-T_s - w_b x Z * h_w)? w_b x Z * h_w may be negligible if sim is to be believed, where did the negative on T_s go?
  void Calculate(const imu::Vector<3>& sat_torque,
    float* const* const wheel_torques) const override;

  // Given a vector of requested wheel torques, PID each motor to that speed by changing
  // and returning the PWM output in the pwm return parameter.
  void Pid_Speed(const float* const* const wheel_torques, const uint32_t dt,
    controller::WheelSpeedPD wpd, uint8_t* const* const pwm) override;

 private:
  const uint8_t kNumWheels = 4;
  uint8_t wheel_pwm_[4];
  const float kWheelMoment[4] = {0, 0, 0, 0};
  const float kSqrt3Div4 = 0.433012701892219;
  imu::Matrix<4> pseudoinverse_;
};

}  // namespace pointing_modes
#endif  // RW_SRC_POINTINGMODES_HPP_
