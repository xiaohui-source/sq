#ifndef MOTOR_CONTROL_IMOTOR_DRIVER_H_
#define MOTOR_CONTROL_IMOTOR_DRIVER_H_

#include <vector>

namespace motor_control
{

class IMotorDriver
{
public:
  virtual ~IMotorDriver() = default;

  virtual bool init() = 0;
  virtual bool open() { return true; }
  virtual bool enable() { return true; }
  virtual bool disable() { return true; }
  virtual void close() {}

  virtual bool read(
      std::vector<double>& state_pos,
      std::vector<double>& state_vel,
      std::vector<double>& state_eff) = 0;

  virtual bool write(
      const std::vector<double>& cmd_pos,
      const std::vector<double>& cmd_vel,
      const std::vector<double>& cmd_eff) = 0;
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_IMOTOR_DRIVER_H_
