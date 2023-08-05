#ifndef RW_SRC_CONTROLLER_HPP_
#define RW_SRC_CONTROLLER_HPP_

namespace controller {
// Standard linear-error PID controller
class PID {
 public:
  PID(const float p, const float i, const float d) : kP(p), kI(i), kD(d),
    prev_err_(0), accumulated_err_(0) {}
  virtual ~PID() = default;
  PID(const PID& other) = default;
  PID& operator=(const PID& other) = default;

  // Based on input parameters, calculates the PID output and returns it as a float.
  // Modifies internal state of error tracking.
  virtual float Compute(const float setpoint, const float curr_pos, const float dt);

  // Resets the state of this controller.
  void Reset();

 protected:
  // The specific algorithm used to calculate the error in Compute()
  virtual float CalcError(const float setpoint, const float curr_pos);
 private:
  const float kP, kI, kD;
  float prev_err_;
  float accumulated_err_;
};
}  // namespace controller
#endif // RW_SRC_CONTROLLER_HPP_
