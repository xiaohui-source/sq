#include "motor_control/CANDriver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <thread>

namespace motor_control
{
namespace
{

constexpr std::uint32_t kDeviceType = VCI_USBCAN2A;
constexpr std::uint32_t kDeviceIndex = 0;
constexpr std::size_t kReceiveBatchSize = 64;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kCommandEpsilon = 1e-6;
constexpr DWORD kOpenDeviceFailure = static_cast<DWORD>(-1);

bool nearly_equal(double lhs, double rhs)
{
  return std::fabs(lhs - rhs) <= kCommandEpsilon;
}

std::string trim_c_string(const CHAR* raw, std::size_t size)
{
  if (raw == nullptr || size == 0)
  {
    return {};
  }

  std::size_t length = 0;
  while (length < size && raw[length] != '\0')
  {
    ++length;
  }

  std::string value(raw, raw + length);
  while (!value.empty() && (value.back() == ' ' || value.back() == '\t'))
  {
    value.pop_back();
  }
  return value;
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
    mode_initialized_(joint_configs_.size(), false),
    zero_reference_pos_(joint_configs_.size(), 0.0),
    zero_reference_initialized_(joint_configs_.size(), false),
    joint_enabled_(joint_configs_.size(), false)
{
}

CANDriver::~CANDriver()
{
  std::cout << "[CAN] ~CANDriver, device_handle_open_=" << device_handle_open_
            << ", device_open_=" << device_open_ << std::endl;
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

  std::cout << "[CAN] open(), initialized_=" << initialized_
            << ", device_handle_open_=" << device_handle_open_
            << ", device_open_=" << device_open_ << std::endl;
  if (!initialized_)
  {
    std::cout << "[CAN] NOT initialized" << std::endl;
    return false;
  }

  if (device_open_)
  {
    std::cout << "[CAN] already open" << std::endl;
    return true;
  }

  if (device_handle_open_)
  {
    std::cout << "[CAN] stale device handle detected before open, skipping duplicate open attempt" << std::endl;
    return false;
  }

  if (can_channel_ > 1)
  {
    std::cout << "[CAN] invalid channel: " << can_channel_ << ", expected 0 or 1" << std::endl;
    return false;
  }

  std::cout << "[CAN] OpenDevice..., type=" << kDeviceType
            << ", device_index=" << kDeviceIndex
            << ", channel=" << can_channel_
            << ", baud=" << baud_rate_ << "kbps" << std::endl;

  const DWORD open_status = VCI_OpenDevice(kDeviceType, kDeviceIndex, 0);
  std::cout << "[CAN] OpenDevice return value=" << open_status << std::endl;
  if (open_status == kOpenDeviceFailure)
  {
    VCI_BOARD_INFO board_info[8] {};
    const DWORD found_devices = VCI_FindUsbDevice2(board_info);

    std::cout << "[CAN] OpenDevice FAILED, return=" << open_status
              << ", device_handle_open_=" << device_handle_open_
              << ", device_open_=" << device_open_
              << ", detected_usb_can_devices=" << found_devices << std::endl;
    if (found_devices == 0)
    {
      std::cout << "[CAN] Hint: check USB access permissions for CANalyst-II (VID:PID 04d8:0053)" << std::endl;
    }
    return false;
  }
  device_handle_open_ = true;
  device_open_ = false;
  std::cout << "[CAN] OpenDevice succeeded, device_handle_open_=" << device_handle_open_
            << ", device_open_=" << device_open_ << std::endl;

  VCI_BOARD_INFO board_info {};
  const DWORD board_status = VCI_ReadBoardInfo(kDeviceType, kDeviceIndex, &board_info);
  if (board_status == STATUS_OK)
  {
    std::cout << "[CAN] Board info: hw_type=" << trim_c_string(board_info.str_hw_Type, sizeof(board_info.str_hw_Type))
              << ", serial=" << trim_c_string(board_info.str_Serial_Num, sizeof(board_info.str_Serial_Num))
              << ", can_count=" << static_cast<int>(board_info.can_Num) << std::endl;
  }
  else
  {
    std::cout << "[CAN] ReadBoardInfo FAILED, return=" << board_status << std::endl;
  }

  VCI_INIT_CONFIG config_can {};
  config_can.AccCode = 0;
  config_can.AccMask = 0xFFFFFFFF;
  config_can.Filter = 1;
  config_can.Mode = 0;
  config_can.Timing0 = static_cast<UCHAR>(baud_to_timing0());
  config_can.Timing1 = static_cast<UCHAR>(baud_to_timing1());

  std::cout << "[CAN] InitCAN..." << std::endl;
  const DWORD init_status = VCI_InitCAN(kDeviceType, kDeviceIndex, can_channel_, &config_can);
  std::cout << "[CAN] InitCAN return value=" << init_status
            << ", device_handle_open_=" << device_handle_open_
            << ", device_open_=" << device_open_ << std::endl;
  if (init_status != STATUS_OK)
  {
    std::cout << "[CAN] InitCAN FAILED, return=" << init_status
              << ", channel=" << can_channel_
              << ", timing0=0x" << std::hex << static_cast<int>(config_can.Timing0)
              << ", timing1=0x" << static_cast<int>(config_can.Timing1)
              << std::dec << std::endl;
    if (device_handle_open_)
    {
      const DWORD close_status = VCI_CloseDevice(kDeviceType, kDeviceIndex);
      std::cout << "[CAN] CloseDevice after InitCAN failure executed, return=" << close_status << std::endl;
      device_handle_open_ = false;
      device_open_ = false;
    }
    else
    {
      std::cout << "[CAN] CloseDevice after InitCAN failure skipped, device_handle_open_="
                << device_handle_open_ << ", device_open_=" << device_open_ << std::endl;
    }
    return false;
  }

  std::cout << "[CAN] StartCAN..." << std::endl;
  const DWORD start_status = VCI_StartCAN(kDeviceType, kDeviceIndex, can_channel_);
  std::cout << "[CAN] StartCAN return value=" << start_status
            << ", device_handle_open_=" << device_handle_open_
            << ", device_open_=" << device_open_ << std::endl;
  if (start_status != STATUS_OK)
  {
    std::cout << "[CAN] StartCAN FAILED, return=" << start_status
              << ", channel=" << can_channel_ << std::endl;
    if (device_handle_open_)
    {
      const DWORD close_status = VCI_CloseDevice(kDeviceType, kDeviceIndex);
      std::cout << "[CAN] CloseDevice after StartCAN failure executed, return=" << close_status << std::endl;
      device_handle_open_ = false;
      device_open_ = false;
    }
    else
    {
      std::cout << "[CAN] CloseDevice after StartCAN failure skipped, device_handle_open_="
                << device_handle_open_ << ", device_open_=" << device_open_ << std::endl;
    }
    return false;
  }

  const DWORD clear_status = VCI_ClearBuffer(kDeviceType, kDeviceIndex, can_channel_);
  if (clear_status != STATUS_OK)
  {
    std::cout << "[CAN] ClearBuffer FAILED, return=" << clear_status << std::endl;
    if (device_handle_open_)
    {
      const DWORD close_status = VCI_CloseDevice(kDeviceType, kDeviceIndex);
      std::cout << "[CAN] CloseDevice after ClearBuffer failure executed, return=" << close_status << std::endl;
      device_handle_open_ = false;
      device_open_ = false;
    }
    else
    {
      std::cout << "[CAN] CloseDevice after ClearBuffer failure skipped, device_handle_open_="
                << device_handle_open_ << ", device_open_=" << device_open_ << std::endl;
    }
    return false;
  }

  device_open_ = true;
  std::cout << "[CAN] OPEN SUCCESS, device_open_=" << device_open_ << std::endl;

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
    if (!set_joint_enabled(joint_index, true))
    {
      return false;
    }
  }

  enabled_ = std::any_of(joint_enabled_.begin(), joint_enabled_.end(), [](bool value) { return value; });
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
    ok = set_joint_enabled(joint_index, false) && ok;
  }

  enabled_ = false;
  return ok;
}

bool CANDriver::enable_joint(const std::string& joint_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!device_open_)
  {
    return false;
  }

  const std::size_t joint_index = find_joint_index_by_name(joint_name);
  if (joint_index >= joint_configs_.size())
  {
    return false;
  }

  const bool ok = set_joint_enabled(joint_index, true);
  enabled_ = std::any_of(joint_enabled_.begin(), joint_enabled_.end(), [](bool value) { return value; });
  return ok;
}

bool CANDriver::disable_joint(const std::string& joint_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!device_open_)
  {
    return false;
  }

  const std::size_t joint_index = find_joint_index_by_name(joint_name);
  if (joint_index >= joint_configs_.size())
  {
    return false;
  }

  const bool ok = set_joint_enabled(joint_index, false);
  enabled_ = std::any_of(joint_enabled_.begin(), joint_enabled_.end(), [](bool value) { return value; });
  return ok;
}

void CANDriver::close()
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::cout << "[CAN] close(), device_handle_open_=" << device_handle_open_
            << ", device_open_=" << device_open_
            << ", enabled_=" << enabled_ << std::endl;
  if (!device_handle_open_)
  {
    std::cout << "[CAN] CloseDevice skipped, device_handle_open_=false" << std::endl;
    device_open_ = false;
    enabled_ = false;
    std::fill(zero_reference_initialized_.begin(), zero_reference_initialized_.end(), false);
    return;
  }

  if (enabled_ && device_open_)
  {
    for (std::size_t joint_index = 0; joint_index < joint_configs_.size(); ++joint_index)
    {
      if (joint_enabled_[joint_index])
      {
        VCI_CAN_OBJ frame {};
        encode_mode_frame(joint_index, 6, frame);
        transmit_frame(frame);
      }
    }
    enabled_ = false;
  }

  if (device_open_)
  {
    const DWORD reset_status = VCI_ResetCAN(kDeviceType, kDeviceIndex, can_channel_);
    std::cout << "[CAN] ResetCAN return=" << reset_status << std::endl;
  }
  else
  {
    std::cout << "[CAN] ResetCAN skipped, device_open_=false" << std::endl;
  }

  const DWORD close_status = VCI_CloseDevice(kDeviceType, kDeviceIndex);
  std::cout << "[CAN] CloseDevice executed, return=" << close_status << std::endl;
  device_handle_open_ = false;
  device_open_ = false;
  std::fill(zero_reference_initialized_.begin(), zero_reference_initialized_.end(), false);
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

  if (!all_zero_references_initialized())
  {
    read_nonblocking_feedback();
    if (!all_zero_references_initialized())
    {
      return true;
    }
  }

  for (std::size_t joint_index = 0; joint_index < joint_configs_.size(); ++joint_index)
  {
    if (!is_joint_enabled(joint_index))
    {
      continue;
    }

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
//写不保存参数寄存器
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
      frame.Data[5] = 0x03;//can总线
      break;
    //内部使能
    case 5:
      frame.Data[3] = 0x10;
      frame.Data[5] = 0x01;
      break;
    //内部失能
    case 6:
      frame.Data[3] = 0x10;
      break;
    //绝对位置协议速度模式
    case 1:
      frame.Data[3] = 0x03;
      frame.Data[5] = 0x06;
      break;
    //速度运行
    case 2:
      frame.Data[3] = 0x03;
      frame.Data[5] = 0x02;
      break;
    //力矩运行
    case 3:
    default:
      frame.Data[3] = 0x03;
      frame.Data[5] = 0x01;
      break;
  }
}
//判断控制方式，对应的物理单位（弧度、速度、力）换算成伺服底层的单位（脉冲、RPM、千分比）并发给驱动器
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
    if (zero_reference_initialized_[joint_index])
    {
      cmd_pos += zero_reference_pos_[joint_index];
    }
    const double output_turns =
        joint.is_revolute ? (cmd_pos / kTwoPi) : (cmd_pos / joint.output_translation_per_rev);
    const double motor_turns =
        output_turns * gear_ratio;
    const std::int32_t counts = static_cast<std::int32_t>(std::llround(motor_turns * 10000.0));//电机编码器分辨率10000,乘以要转的圈数
    frame.Data[1] = 0x3c;
    frame.Data[3] = 0x99;//安全限速
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
//读取并且解析反馈帧，计算出电机的绝对位置、速度和力矩，并存储在joint_states_中
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
  const double absolute_position =
      joint.is_revolute ? (output_turns * kTwoPi) : (output_turns * joint.output_translation_per_rev);
  capture_zero_reference_if_needed(joint_index, absolute_position);
  joint_states_[joint_index].position =
      absolute_position -
      (zero_reference_initialized_[joint_index] ? zero_reference_pos_[joint_index] : 0.0);

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

  if (frame.Data[1] == 0xCB)
  {
    const std::uint16_t status_word =
        static_cast<std::uint16_t>(frame.Data[2]) |
        (static_cast<std::uint16_t>(frame.Data[3]) << 8);
    const bool previous_enabled = joint_states_[joint_index].actually_enabled;
    const bool previous_fault = joint_states_[joint_index].has_fault;
    const bool current_enabled = (status_word & (static_cast<std::uint16_t>(1) << 1)) != 0;
    const bool current_fault = (status_word & (static_cast<std::uint16_t>(1) << 6)) != 0;

    if (!previous_enabled && current_enabled)
    {
      std::cout << "\033[32m[OK] [Joint " << joint.name
                << "] Motor Enabled Successfully!\033[0m" << std::endl;
    }
    else if (previous_enabled && !current_enabled)
    {
      std::cout << "\033[33m[WARN] [Joint " << joint.name
                << "] Motor Disabled.\033[0m" << std::endl;
    }

    if (!previous_fault && current_fault)
    {
      std::cerr << "\033[31m[FAULT] [Joint " << joint.name
                << "] Motor FAULT / ALARM triggered! Disabled automatically.\033[0m" << std::endl;
    }
    else if (previous_fault && !current_fault)
    {
      std::cout << "\033[32m[RECOVER] [Joint " << joint.name
                << "] Motor fault cleared.\033[0m" << std::endl;
    }

    joint_states_[joint_index].actually_enabled = current_enabled;
    joint_states_[joint_index].has_fault = current_fault;
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
//数据接收并进行调度
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
//控制模式仲裁，优先级：力矩>速度>位置
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

bool CANDriver::all_zero_references_initialized() const
{
  bool has_enabled_joint = false;
  for (std::size_t joint_index = 0; joint_index < zero_reference_initialized_.size(); ++joint_index)
  {
    if (!joint_enabled_[joint_index])
    {
      continue;
    }
    has_enabled_joint = true;
    if (!zero_reference_initialized_[joint_index])
    {
      return false;
    }
  }
  return has_enabled_joint;
}

bool CANDriver::is_joint_enabled(std::size_t joint_index) const
{
  return joint_index < joint_enabled_.size() && joint_enabled_[joint_index];
}

void CANDriver::capture_zero_reference_if_needed(std::size_t joint_index, double absolute_position)
{
  if (joint_index >= zero_reference_initialized_.size() || zero_reference_initialized_[joint_index])
  {
    return;
  }

  zero_reference_pos_[joint_index] = absolute_position;
  zero_reference_initialized_[joint_index] = true;
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

std::size_t CANDriver::find_joint_index_by_name(const std::string& joint_name) const
{
  for (std::size_t joint_index = 0; joint_index < joint_configs_.size(); ++joint_index)
  {
    if (joint_configs_[joint_index].name == joint_name)
    {
      return joint_index;
    }
  }
  return joint_configs_.size();
}

bool CANDriver::set_joint_enabled(std::size_t joint_index, bool enabled)
{
  if (joint_index >= joint_configs_.size())
  {
    return false;
  }

  if (joint_enabled_[joint_index] == enabled)
  {
    return true;
  }

  VCI_CAN_OBJ frame {};
  if (enabled)
  {
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

    mode_initialized_[joint_index] = false;
    zero_reference_initialized_[joint_index] = false;
    for (int attempt = 0; attempt < 20 && !zero_reference_initialized_[joint_index]; ++attempt)
    {
      read_nonblocking_feedback();
      if (!zero_reference_initialized_[joint_index])
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    joint_enabled_[joint_index] = true;
    return true;
  }

  encode_mode_frame(joint_index, 6, frame);
  if (!transmit_frame(frame))
  {
    return false;
  }

  joint_enabled_[joint_index] = false;
  mode_initialized_[joint_index] = false;
  zero_reference_initialized_[joint_index] = false;
  return true;
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
