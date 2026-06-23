#include "motor_control/CANDriver.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace motor_control
{
namespace
{

constexpr std::uint32_t kDeviceType = VCI_USBCAN2;
constexpr std::uint32_t kDeviceIndex = 0;
constexpr std::size_t kReceiveBatchSize = 64;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kCommandEpsilon = 1e-6;

bool nearly_equal(double lhs, double rhs)
{
  return std::fabs(lhs - rhs) <= kCommandEpsilon;
}

}  // namespace

CANDriver::CANDriver(
    std::vector<JointConfig> joint_configs,
    std::uint32_t can_channel,
    std::uint32_t baud_rate)
  : joint_configs_(std::move(joint_configs)),
    can_channel_(can_channel),
    baud_rate_(baud_rate),
    joint_states_(joint_configs_.size()),
    last_cmd_pos_(joint_configs_.size(), 0.0),
    last_cmd_vel_(joint_configs_.size(), 0.0),
    last_cmd_eff_(joint_configs_.size(), 0.0),
    last_modes_(joint_configs_.size(), ControlMode::kPosition),
    mode_initialized_(joint_configs_.size(), false)
{
}

CANDriver::~CANDriver()
{
  close();
}

bool CANDriver::init()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (joint_configs_.empty())
  {
    return false;
  }

  for (const auto& joint_config : joint_configs_)
  {
    if (joint_config.motor_id == 0 || joint_config.force_max <= 0.0 || joint_config.gear_ratio == 0.0)
    {
      return false;
    }
    if (!joint_config.is_revolute && joint_config.output_translation_per_rev == 0.0)
    {
      return false;
    }
  }

  initialized_ = true;
  return true;
}

bool CANDriver::open()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_)
  {
    return false;
  }

  if (device_open_)
  {
    return true;
  }

  if (VCI_OpenDevice(kDeviceType, kDeviceIndex, 0) != STATUS_OK)
  {
    return false;
  }

  VCI_INIT_CONFIG config_can {};
  config_can.AccCode = 0;
  config_can.AccMask = 0xFFFFFFFF;
  config_can.Filter = 1;
  config_can.Mode = 0;
  config_can.Timing0 = static_cast<UCHAR>(baud_to_timing0());
  config_can.Timing1 = static_cast<UCHAR>(baud_to_timing1());

  if (VCI_InitCAN(kDeviceType, kDeviceIndex, can_channel_, &config_can) != STATUS_OK)
  {
    VCI_CloseDevice(kDeviceType, kDeviceIndex);
    return false;
  }

  if (VCI_StartCAN(kDeviceType, kDeviceIndex, can_channel_) != STATUS_OK)
  {
    VCI_CloseDevice(kDeviceType, kDeviceIndex);
    return false;
  }

  VCI_ClearBuffer(kDeviceType, kDeviceIndex, can_channel_);
  device_open_ = true;
  return true;
}

bool CANDriver::enable()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!device_open_)
  {
    return false;
  }

  for (std::size_t joint_index = 0; joint_index < joint_configs_.size(); ++joint_index)
  {
    VCI_CAN_OBJ frame {};
    encode_mode_frame(joint_index, 4, frame);
    if (!transmit_frame(frame))
    {
      return false;
    }
    encode_mode_frame(joint_index, 5, frame);
    if (!transmit_frame(frame))
    {
      return false;
    }
  }

  enabled_ = true;
  std::fill(mode_initialized_.begin(), mode_initialized_.end(), false);
  return true;
}

bool CANDriver::disable()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!device_open_)
  {
    return true;
  }

  bool ok = true;
  for (std::size_t joint_index = 0; joint_index < joint_configs_.size(); ++joint_index)
  {
    VCI_CAN_OBJ frame {};
    encode_mode_frame(joint_index, 6, frame);
    ok = transmit_frame(frame) && ok;
  }

  enabled_ = false;
  return ok;
}

void CANDriver::close()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!device_open_)
  {
    return;
  }

  if (enabled_)
  {
    for (std::size_t joint_index = 0; joint_index < joint_configs_.size(); ++joint_index)
    {
      VCI_CAN_OBJ frame {};
      encode_mode_frame(joint_index, 6, frame);
      transmit_frame(frame);
    }
    enabled_ = false;
  }

  VCI_ResetCAN(kDeviceType, kDeviceIndex, can_channel_);
  VCI_CloseDevice(kDeviceType, kDeviceIndex);
  device_open_ = false;
}

bool CANDriver::read(
    std::vector<double>& state_pos,
    std::vector<double>& state_vel,
    std::vector<double>& state_eff)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!device_open_)
  {
    return false;
  }

  if (!read_nonblocking_feedback())
  {
    return false;
  }

  state_pos.resize(joint_states_.size());
  state_vel.resize(joint_states_.size());
  state_eff.resize(joint_states_.size());
  for (std::size_t joint_index = 0; joint_index < joint_states_.size(); ++joint_index)
  {
    state_pos[joint_index] = joint_states_[joint_index].position;
    state_vel[joint_index] = joint_states_[joint_index].velocity;
    state_eff[joint_index] = joint_states_[joint_index].effort;
  }
  return true;
}

bool CANDriver::write(
    const std::vector<double>& cmd_pos,
    const std::vector<double>& cmd_vel,
    const std::vector<double>& cmd_eff)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_ || cmd_pos.size() != joint_configs_.size() ||
      cmd_vel.size() != joint_configs_.size() || cmd_eff.size() != joint_configs_.size())
  {
    return false;
  }

  for (std::size_t joint_index = 0; joint_index < joint_configs_.size(); ++joint_index)
  {
    const ControlMode desired_mode =
        choose_mode(joint_index, cmd_pos[joint_index], cmd_vel[joint_index], cmd_eff[joint_index]);
    if (!mode_initialized_[joint_index] || desired_mode != last_modes_[joint_index])
    {
      VCI_CAN_OBJ mode_frame {};
      encode_mode_frame(joint_index, static_cast<int>(desired_mode), mode_frame);
      if (!transmit_frame(mode_frame))
      {
        return false;
      }
      last_modes_[joint_index] = desired_mode;
      mode_initialized_[joint_index] = true;
    }

    VCI_CAN_OBJ command_frame {};
    encode_control_frame(
        joint_index, desired_mode, cmd_pos[joint_index], cmd_vel[joint_index], cmd_eff[joint_index], command_frame);
    if (!transmit_frame(command_frame))
    {
      return false;
    }

    last_cmd_pos_[joint_index] = cmd_pos[joint_index];
    last_cmd_vel_[joint_index] = cmd_vel[joint_index];
    last_cmd_eff_[joint_index] = cmd_eff[joint_index];
  }

  return true;
}

bool CANDriver::transmit_frame(const VCI_CAN_OBJ& frame)
{
  VCI_CAN_OBJ frame_copy = frame;
  return VCI_Transmit(kDeviceType, kDeviceIndex, can_channel_, &frame_copy, 1) == STATUS_OK;
}

void CANDriver::encode_mode_frame(std::size_t joint_index, int mode, VCI_CAN_OBJ& frame) const
{
  std::memset(&frame, 0, sizeof(frame));
  frame.ID = 0x100 + joint_configs_[joint_index].motor_id;
  frame.SendType = 1;
  frame.RemoteFlag = 0;
  frame.ExternFlag = 0;
  frame.DataLen = 8;
  frame.Data[1] = 0x24;

  switch (mode)
  {
    case 4:
      frame.Data[5] = 0x03;
      break;
    case 5:
      frame.Data[3] = 0x10;
      frame.Data[5] = 0x01;
      break;
    case 6:
      frame.Data[3] = 0x10;
      break;
    case 1:
      frame.Data[3] = 0x03;
      frame.Data[5] = 0x06;
      break;
    case 2:
      frame.Data[3] = 0x03;
      frame.Data[5] = 0x02;
      break;
    case 3:
    default:
      frame.Data[3] = 0x03;
      frame.Data[5] = 0x01;
      break;
  }
}

void CANDriver::encode_control_frame(
    std::size_t joint_index,
    ControlMode mode,
    double cmd_pos,
    double cmd_vel,
    double cmd_eff,
    VCI_CAN_OBJ& frame) const
{
  const auto& joint = joint_configs_[joint_index];
  const double gear_ratio = clamp_gear_ratio(joint.gear_ratio);

  std::memset(&frame, 0, sizeof(frame));
  frame.ID = 0x100 + joint.motor_id;
  frame.SendType = 1;
  frame.RemoteFlag = 0;
  frame.ExternFlag = 0;
  frame.DataLen = 8;

  if (mode == ControlMode::kPosition)
  {
    cmd_pos = clamp_command_position(joint_index, cmd_pos);
    const double output_turns =
        joint.is_revolute ? (cmd_pos / kTwoPi) : (cmd_pos / joint.output_translation_per_rev);
    const double motor_turns =
        output_turns * gear_ratio;
    const std::int32_t counts = static_cast<std::int32_t>(std::llround(motor_turns * 10000.0));
    frame.Data[1] = 0x3c;
    frame.Data[3] = 0x99;
    frame.Data[4] = static_cast<BYTE>((counts >> 24) & 0xFF);
    frame.Data[5] = static_cast<BYTE>((counts >> 16) & 0xFF);
    frame.Data[6] = static_cast<BYTE>((counts >> 8) & 0xFF);
    frame.Data[7] = static_cast<BYTE>(counts & 0xFF);
    return;
  }

  frame.Data[1] = 0x28;
  if (mode == ControlMode::kVelocity)
  {
    const double motor_rpm =
        joint.is_revolute
            ? (cmd_vel * gear_ratio * 60.0 / kTwoPi)
            : (cmd_vel * gear_ratio * 60.0 / joint.output_translation_per_rev);
    const std::int16_t rpm_raw = static_cast<std::int16_t>(std::llround(motor_rpm));
    frame.Data[4] = static_cast<BYTE>((rpm_raw >> 8) & 0xFF);
    frame.Data[5] = static_cast<BYTE>(rpm_raw & 0xFF);
    return;
  }

  frame.Data[3] = 0x01;
  const double normalized = (cmd_eff / joint.force_max) * 1000.0 / gear_ratio;
  const std::int16_t torque_raw = static_cast<std::int16_t>(std::llround(normalized));
  frame.Data[4] = static_cast<BYTE>((torque_raw >> 8) & 0xFF);
  frame.Data[5] = static_cast<BYTE>(torque_raw & 0xFF);
}

void CANDriver::decode_feedback_frame(std::size_t joint_index, const VCI_CAN_OBJ& frame)
{
  const auto& joint = joint_configs_[joint_index];
  const double gear_ratio = clamp_gear_ratio(joint.gear_ratio);

  const std::int32_t raw_position =
      static_cast<std::int32_t>(
          static_cast<std::uint32_t>(frame.Data[7]) |
          (static_cast<std::uint32_t>(frame.Data[6]) << 8) |
          (static_cast<std::uint32_t>(frame.Data[5]) << 16) |
          (static_cast<std::uint32_t>(frame.Data[4]) << 24));

  const double motor_turns = static_cast<double>(raw_position) / 10000.0;
  const double output_turns = motor_turns / gear_ratio;
  joint_states_[joint_index].position =
      joint.is_revolute ? (output_turns * kTwoPi) : (output_turns * joint.output_translation_per_rev);

  if (frame.Data[1] == 0xCA)
  {
    const std::int16_t raw_torque =
        static_cast<std::int16_t>(
            static_cast<std::uint16_t>(frame.Data[3]) |
            (static_cast<std::uint16_t>(frame.Data[2]) << 8));
    joint_states_[joint_index].effort =
        -static_cast<double>(raw_torque) / 1000.0 * joint.force_max * gear_ratio;
    return;
  }

  const std::int16_t raw_rpm =
      static_cast<std::int16_t>(
          static_cast<std::uint16_t>(frame.Data[3]) |
          (static_cast<std::uint16_t>(frame.Data[2]) << 8));
  const double motor_rps = static_cast<double>(raw_rpm) / 60.0;
  const double output_rps = motor_rps / gear_ratio;
  joint_states_[joint_index].velocity =
      joint.is_revolute ? (output_rps * kTwoPi) : (output_rps * joint.output_translation_per_rev);
}

bool CANDriver::read_nonblocking_feedback()
{
  std::vector<VCI_CAN_OBJ> frames(kReceiveBatchSize);
  const auto received = VCI_Receive(
      kDeviceType, kDeviceIndex, can_channel_, frames.data(), static_cast<UINT>(frames.size()), 0);
  if (received == static_cast<ULONG>(-1))
  {
    return false;
  }

  for (ULONG frame_index = 0; frame_index < received; ++frame_index)
  {
    const std::size_t joint_index = find_joint_index_by_frame_id(frames[frame_index].ID);
    if (joint_index >= joint_configs_.size())
    {
      continue;
    }
    decode_feedback_frame(joint_index, frames[frame_index]);
  }

  return true;
}

std::size_t CANDriver::find_joint_index_by_frame_id(std::uint32_t frame_id) const
{
  for (std::size_t joint_index = 0; joint_index < joint_configs_.size(); ++joint_index)
  {
    if (frame_id == 0x600 + joint_configs_[joint_index].motor_id)
    {
      return joint_index;
    }
  }
  return joint_configs_.size();
}

CANDriver::ControlMode CANDriver::choose_mode(
    std::size_t joint_index,
    double cmd_pos,
    double cmd_vel,
    double cmd_eff) const
{
  const bool effort_active =
      !nearly_equal(cmd_eff, 0.0) ||
      (!nearly_equal(cmd_eff, last_cmd_eff_[joint_index]) && last_modes_[joint_index] == ControlMode::kEffort);
  if (effort_active)
  {
    return ControlMode::kEffort;
  }

  const bool velocity_active =
      !nearly_equal(cmd_vel, 0.0) ||
      (!nearly_equal(cmd_vel, last_cmd_vel_[joint_index]) && nearly_equal(cmd_eff, 0.0));
  if (velocity_active)
  {
    return ControlMode::kVelocity;
  }

  return ControlMode::kPosition;
}

double CANDriver::clamp_gear_ratio(double gear_ratio) const
{
  if (std::fabs(gear_ratio) < std::numeric_limits<double>::epsilon())
  {
    return 1.0;
  }
  return gear_ratio;
}

double CANDriver::clamp_command_position(std::size_t joint_index, double position) const
{
  const auto& joint = joint_configs_[joint_index];
  const double minimum = std::min(joint.position_min, joint.position_max);
  const double maximum = std::max(joint.position_min, joint.position_max);
  return std::clamp(position, minimum, maximum);
}

std::uint32_t CANDriver::baud_to_timing0() const
{
  return 0x00;
}

std::uint32_t CANDriver::baud_to_timing1() const
{
  if (baud_rate_ == 1000)
  {
    return 0x14;
  }
  if (baud_rate_ == 500)
  {
    return 0x1c;
  }
  return 0x14;
}

}  // namespace motor_control
