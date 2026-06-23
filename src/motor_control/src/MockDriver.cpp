#include "motor_control/MockDriver.h"

#include <algorithm>
#include <mutex>

namespace motor_control
{

namespace
{

constexpr double kDefaultDt = 0.001;

}  // namespace

MockDriver::MockDriver(std::size_t joint_count)
  : MockDriver(joint_count, kDefaultDt)
{
}

MockDriver::MockDriver(std::size_t joint_count, double dt)
  : joint_count_(joint_count),
    dt_(dt),
    state_pos_(joint_count, 0.0),
    state_vel_(joint_count, 0.0),
    state_eff_(joint_count, 0.0),
    cmd_pos_(joint_count, 0.0),
    cmd_vel_(joint_count, 0.0),
    cmd_eff_(joint_count, 0.0)
{
}

bool MockDriver::init()
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::fill(state_pos_.begin(), state_pos_.end(), 0.0);
  std::fill(state_vel_.begin(), state_vel_.end(), 0.0);
  std::fill(state_eff_.begin(), state_eff_.end(), 0.0);
  std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
  std::fill(cmd_eff_.begin(), cmd_eff_.end(), 0.0);
  return true;
}

bool MockDriver::open()
{
  return true;
}

bool MockDriver::enable()
{
  return true;
}

bool MockDriver::disable()
{
  return true;
}

void MockDriver::close()
{
}

bool MockDriver::read(
    std::vector<double>& state_pos,
    std::vector<double>& state_vel,
    std::vector<double>& state_eff)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // In mock mode, mirror the commanded values directly back as feedback.
  // Some controllers publish position-only commands, so integrating velocity
  // would otherwise leave position feedback stuck at zero.
  state_pos_ = cmd_pos_;
  state_vel_ = cmd_vel_;
  state_eff_ = cmd_eff_;

  state_pos = state_pos_;
  state_vel = state_vel_;
  state_eff = state_eff_;
  return true;
}

bool MockDriver::write(
    const std::vector<double>& cmd_pos,
    const std::vector<double>& cmd_vel,
    const std::vector<double>& cmd_eff)
{
  std::lock_guard<std::mutex> lock(mutex_);

  cmd_pos_ = cmd_pos;
  cmd_vel_ = cmd_vel;
  cmd_eff_ = cmd_eff;
  return true;
}

}  // namespace motor_control
