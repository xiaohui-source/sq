#ifndef MOTOR_CONTROL_MOCK_DRIVER_H_
#define MOTOR_CONTROL_MOCK_DRIVER_H_

#include "motor_control/IMotorDriver.h"

#include <cstddef>
#include <mutex>
#include <vector>

namespace motor_control
{

class MockDriver : public IMotorDriver
{
public:
  explicit MockDriver(std::size_t joint_count);
  MockDriver(std::size_t joint_count, double dt);

  bool init() override;
  bool open() override;
  bool enable() override;
  bool disable() override;
  void close() override;
  bool read(
      std::vector<double>& state_pos,
      std::vector<double>& state_vel,
      std::vector<double>& state_eff) override;
  bool write(
      const std::vector<double>& cmd_pos,
      const std::vector<double>& cmd_vel,
      const std::vector<double>& cmd_eff) override;

private:
  const std::size_t joint_count_;
  const double dt_;

  mutable std::mutex mutex_;
  std::vector<double> state_pos_;
  std::vector<double> state_vel_;
  std::vector<double> state_eff_;
  std::vector<double> cmd_pos_;
  std::vector<double> cmd_vel_;
  std::vector<double> cmd_eff_;
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_MOCK_DRIVER_H_
