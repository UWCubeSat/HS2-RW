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
  // changing and returning the PWM output.
  virtual uint8_t* Pid_Speed(const float* const * const wheel_torques) = 0;
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
  // and returning the PWM output in the return parameter.
  uint8_t* Pid_Speed(const float* const* const wheel_torques) override;

  // TODO make this not stl
  // Z, the matrix of wheel torque distributions
  const Eigen::Matrix<float, 3, 4> kWheelTorqueMatrix = (1/std::sqrt(3)) *
    (Eigen::Matrix<float, 3, 4>() << std::sqrt(2), 0, -std::sqrt(2), 0, 0,
    std::sqrt(2), 0, -std::sqrt(2), 1, 1, 1, 1).finished();
  // Z+, the pseudoinverse of Z such that Z * Z+ = Z+ * Z = I in R3x3 and R4x4 respectively
  const Eigen::Matrix<float, 4, 3> kPseudoinverse =
    kWheelTorqueMatrix.completeOrthogonalDecomposition().pseudoInverse();
 private:
  uint8_t wheel_pwm_[4];
};

}  // namespace pointing_modes
#endif  // RW_SRC_POINTINGMODES_HPP_
