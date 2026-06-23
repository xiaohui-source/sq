#include "motor_control/CANDriver.h"
#include "motor_control/motor_control_hardware_interface.h"
#include "motor_control/MockDriver.h"
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace motor_control
{
namespace
{

constexpr std::array<const char*, 3> kInterfaceNames = {
    "position",
    "velocity",
    "effort",
};
constexpr double kLegacyTransRadiusMeters = 0.012;
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

bool has_interface(
    const std::vector<hardware_interface::InterfaceInfo>& interfaces,
    const char* interface_name)
{
  return std::any_of(
      interfaces.begin(), interfaces.end(),
      [interface_name](const hardware_interface::InterfaceInfo& interface_info) {
        return interface_info.name == interface_name;
      });
}

std::string to_lower(std::string value)
{
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
  return value;
}

bool parse_bool_parameter(const std::string& value)
{
  const auto normalized_value = to_lower(value);
  return normalized_value == "true" ||
         normalized_value == "1" ||
         normalized_value == "yes" ||
         normalized_value == "on";
}

bool parse_int_parameter(const std::string& value, int& output)
{
  try
  {
    output = std::stoi(value);
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

bool parse_double_parameter(const std::string& value, double& output)
{
  try
  {
    output = std::stod(value);
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

std::string read_joint_type_parameter(const hardware_interface::ComponentInfo& joint)
{
  const auto parameter_it = joint.parameters.find("joint_type");
  if (parameter_it == joint.parameters.end())
  {
    return "revolute";
  }
  return to_lower(parameter_it->second);
}

template <typename NumericT>
bool read_joint_parameter(
    const hardware_interface::ComponentInfo& joint,
    const char* parameter_name,
    NumericT& output);

template <>
bool read_joint_parameter<int>(
    const hardware_interface::ComponentInfo& joint,
    const char* parameter_name,
    int& output)
{
  const auto parameter_it = joint.parameters.find(parameter_name);
  if (parameter_it == joint.parameters.end())
  {
    return false;
  }
  return parse_int_parameter(parameter_it->second, output);
}

template <>
bool read_joint_parameter<double>(
    const hardware_interface::ComponentInfo& joint,
    const char* parameter_name,
    double& output)
{
  const auto parameter_it = joint.parameters.find(parameter_name);
  if (parameter_it == joint.parameters.end())
  {
    return false;
  }
  return parse_double_parameter(parameter_it->second, output);
}

}  // namespace

MotorControlHardwareInterface::MotorControlHardwareInterface()
  : state_pos_(kJointCount, 0.0),
    state_vel_(kJointCount, 0.0),
    state_eff_(kJointCount, 0.0),
    cmd_pos_(kJointCount, 0.0),
    cmd_vel_(kJointCount, 0.0),
    cmd_eff_(kJointCount, 0.0),
    logger_(rclcpp::get_logger("MotorControlHardwareInterface"))
{
}

MotorControlHardwareInterface::~MotorControlHardwareInterface() = default;

hardware_interface::CallbackReturn MotorControlHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(logger_, "Initializing Motor Control Hardware Interface");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
        logger_, "Expected exactly %zu joints in ros2_control hardware info, got %zu",
        kJointCount, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(state_pos_.begin(), state_pos_.end(), 0.0);
  std::fill(state_vel_.begin(), state_vel_.end(), 0.0);
  std::fill(state_eff_.begin(), state_eff_.end(), 0.0);
  std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
  std::fill(cmd_eff_.begin(), cmd_eff_.end(), 0.0);

  for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
  {
    const auto& joint = info_.joints[joint_index];
    joint_names_[joint_index] = joint.name;

    for (const auto* interface_name : kInterfaceNames)
    {
      if (!has_interface(joint.state_interfaces, interface_name))
      {
        RCLCPP_ERROR(
            logger_, "Joint '%s' is missing required state interface '%s'",
            joint.name.c_str(), interface_name);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!has_interface(joint.command_interfaces, interface_name))
      {
        RCLCPP_ERROR(
            logger_, "Joint '%s' is missing required command interface '%s'",
            joint.name.c_str(), interface_name);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  const auto use_mock_parameter = info_.hardware_parameters.find("use_mock");
  const bool use_mock =
      use_mock_parameter != info_.hardware_parameters.end() &&
      parse_bool_parameter(use_mock_parameter->second);

  int can_channel = 0;
  const auto can_channel_parameter = info_.hardware_parameters.find("can_channel");
  if (can_channel_parameter != info_.hardware_parameters.end() &&
      !parse_int_parameter(can_channel_parameter->second, can_channel))
  {
    RCLCPP_ERROR(logger_, "Invalid hardware parameter 'can_channel': '%s'", can_channel_parameter->second.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  int baud_rate = 1000;
  const auto baud_rate_parameter = info_.hardware_parameters.find("baud_rate");
  if (baud_rate_parameter != info_.hardware_parameters.end() &&
      !parse_int_parameter(baud_rate_parameter->second, baud_rate))
  {
    RCLCPP_ERROR(logger_, "Invalid hardware parameter 'baud_rate': '%s'", baud_rate_parameter->second.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (use_mock)
  {
    driver_ = std::make_unique<MockDriver>(kJointCount);
  }
  else
  {
    std::vector<CANDriver::JointConfig> joint_configs;
    joint_configs.reserve(kJointCount);
    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
    {
      const auto& joint = info_.joints[joint_index];
      int motor_id = 0;
      double gear_ratio = 0.0;
      double force_max = 0.0;
      double soft_limit_min = 0.0;
      double soft_limit_max = 0.0;
      double output_translation_per_rev = 0.0;
      const std::string joint_type = read_joint_type_parameter(joint);

      if (!read_joint_parameter(joint, "motor_id", motor_id))
      {
        RCLCPP_ERROR(logger_, "Joint '%s' is missing valid 'motor_id' parameter", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!read_joint_parameter(joint, "gear_ratio", gear_ratio))
      {
        RCLCPP_ERROR(logger_, "Joint '%s' is missing valid 'gear_ratio' parameter", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!read_joint_parameter(joint, "force_max", force_max))
      {
        RCLCPP_ERROR(logger_, "Joint '%s' is missing valid 'force_max' parameter", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!read_joint_parameter(joint, "soft_limit_min", soft_limit_min))
      {
        RCLCPP_ERROR(logger_, "Joint '%s' is missing valid 'soft_limit_min' parameter", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!read_joint_parameter(joint, "soft_limit_max", soft_limit_max))
      {
        RCLCPP_ERROR(logger_, "Joint '%s' is missing valid 'soft_limit_max' parameter", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      CANDriver::JointConfig joint_config;
      joint_config.name = joint.name;
      joint_config.motor_id = static_cast<std::uint32_t>(motor_id);
      joint_config.gear_ratio = gear_ratio;
      joint_config.force_max = force_max;
      joint_config.is_revolute = joint_type != "prismatic";
      joint_config.position_min = soft_limit_min;
      joint_config.position_max = soft_limit_max;
      if (!joint_config.is_revolute)
      {
        const auto output_translation_it = joint.parameters.find("output_translation_per_rev");
        if (output_translation_it != joint.parameters.end())
        {
          if (!parse_double_parameter(output_translation_it->second, output_translation_per_rev))
          {
            RCLCPP_ERROR(
                logger_, "Joint '%s' has invalid 'output_translation_per_rev' parameter", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
          }
        }
        else
        {
          output_translation_per_rev = kTwoPi * kLegacyTransRadiusMeters;
        }
        joint_config.output_translation_per_rev = output_translation_per_rev;
      }
      joint_configs.push_back(joint_config);
    }

    driver_ = std::make_unique<CANDriver>(
        std::move(joint_configs),
        static_cast<std::uint32_t>(can_channel),
        static_cast<std::uint32_t>(baud_rate));
  }

  if (!driver_->init())
  {
    RCLCPP_ERROR(logger_, "Failed to initialize motor driver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Initialized with %zu joints", joint_names_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorControlHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(logger_, "Configuring Motor Control Hardware Interface");
  if (!driver_->open())
  {
    RCLCPP_ERROR(logger_, "Failed to open motor driver");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorControlHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(logger_, "Activating Motor Control Hardware Interface");
  if (!driver_->enable())
  {
    RCLCPP_ERROR(logger_, "Failed to enable motor driver");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorControlHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(logger_, "Deactivating Motor Control Hardware Interface");
  if (!driver_->disable())
  {
    RCLCPP_ERROR(logger_, "Failed to disable motor driver");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorControlHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(logger_, "Shutting down Motor Control Hardware Interface");
  if (driver_)
  {
    driver_->disable();
    driver_->close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorControlHardwareInterface::on_error(
    const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_ERROR(logger_, "Motor Control Hardware Interface error detected");
  if (driver_)
  {
    driver_->disable();
    driver_->close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MotorControlHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kJointCount * kInterfaceCount);

  if (state_pos_.size() != kJointCount || state_vel_.size() != kJointCount ||
      state_eff_.size() != kJointCount)
  {
    RCLCPP_FATAL(
        logger_, "State storage size mismatch: pos=%zu vel=%zu eff=%zu expected=%zu",
        state_pos_.size(), state_vel_.size(), state_eff_.size(), kJointCount);
    return state_interfaces;
  }

  for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
  {
    const auto& joint_name = joint_names_[joint_index];
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name, kInterfaceNames[kPositionInterface],
        &state_pos_[joint_index]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name, kInterfaceNames[kVelocityInterface],
        &state_vel_[joint_index]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name, kInterfaceNames[kEffortInterface],
        &state_eff_[joint_index]));
  }

  RCLCPP_INFO(logger_, "Exported %zu state interfaces", state_interfaces.size());
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MotorControlHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(kJointCount * kInterfaceCount);

  if (cmd_pos_.size() != kJointCount || cmd_vel_.size() != kJointCount ||
      cmd_eff_.size() != kJointCount)
  {
    RCLCPP_FATAL(
        logger_, "Command storage size mismatch: pos=%zu vel=%zu eff=%zu expected=%zu",
        cmd_pos_.size(), cmd_vel_.size(), cmd_eff_.size(), kJointCount);
    return command_interfaces;
  }

  for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
  {
    const auto& joint_name = joint_names_[joint_index];
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_name, kInterfaceNames[kPositionInterface],
        &cmd_pos_[joint_index]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_name, kInterfaceNames[kVelocityInterface],
        &cmd_vel_[joint_index]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_name, kInterfaceNames[kEffortInterface],
        &cmd_eff_[joint_index]));
  }

  RCLCPP_INFO(logger_, "Exported %zu command interfaces", command_interfaces.size());
  return command_interfaces;
}

hardware_interface::return_type MotorControlHardwareInterface::read(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
  (void)time;
  (void)period;

  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  if (!driver_->read(state_pos_, state_vel_, state_eff_))
  {
    RCLCPP_ERROR(logger_, "Driver read failed");
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO_THROTTLE(
      logger_, steady_clock, 1000,
      "read joint[0]=%s state: pos=%.6f vel=%.6f eff=%.6f",
      joint_names_[0].c_str(), state_pos_[0], state_vel_[0], state_eff_[0]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorControlHardwareInterface::write(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
  (void)time;
  (void)period;

  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  RCLCPP_INFO_THROTTLE(
      logger_, steady_clock, 1000,
      "write joint[0]=%s command: pos=%.6f vel=%.6f eff=%.6f",
      joint_names_[0].c_str(), cmd_pos_[0], cmd_vel_[0], cmd_eff_[0]);

  if (!driver_->write(cmd_pos_, cmd_vel_, cmd_eff_))
  {
    RCLCPP_ERROR(logger_, "Driver write failed");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace motor_control

PLUGINLIB_EXPORT_CLASS(motor_control::MotorControlHardwareInterface,
                       hardware_interface::SystemInterface)
