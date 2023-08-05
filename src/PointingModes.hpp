#ifndef RW_SRC_POINTINGMODES_HPP_
#define RW_SRC_POINTINGMODES_HPP_

#include <eigen3/Eigen/Dense>
#include <stdint.h>
#include <cmath>

namespace pointing_modes {

class PointingMode {
 public:
  PointingMode() = default;
  virtual ~PointingMode() = default;
  PointingMode(const PointingMode& other) = default;
  PointingMode& operator=(const PointingMode& other) = default;

  // Given a vector for a requested satellite torque, convert it to a vector
  // of wheel torques and return it in column vector form. The vector's
  // dimensions will be equal to the number of wheels the PointingMode instance operates for.
  virtual void Calculate(const Eigen::Vector<float, 3>& sat_torque,
    Eigen::Vector<float, Eigen::Dynamic>* const wheel_torques) const = 0;

  // Given a vector of requested wheel torques, PID each motor to that speed by changing
  // and returning the PWM output in the return parameter.
  virtual void Pid_Speed(const Eigen::Vector<float, Eigen::Dynamic>& wheel_torques,
    uint8_t* const pwms) = 0;
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
  virtual void Calculate(const Eigen::Vector<float, 3>& sat_torque,
    Eigen::Vector<float, Eigen::Dynamic>* const wheel_torques) const = 0;

  // Given a vector of requested wheel torques, PID each motor to that speed by changing
  // and returning the PWM output in the return parameter.
  void Pid_Speed(const Eigen::Vector<float, Eigen::Dynamic>& wheel_torques,
    uint8_t* const pwms) override;

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
