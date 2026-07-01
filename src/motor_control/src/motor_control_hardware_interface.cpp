#include "motor_control/CANDriver.h"
#include "motor_control/motor_control_hardware_interface.h"
#include "motor_control/MockDriver.h"
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace motor_control
{
  namespace
  {

    constexpr std::array<const char *, 3> kInterfaceNames = {
        "position",
        "velocity",
        "effort",
    };
    constexpr double kLegacyTransRadiusMeters = 0.012;
    constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

    bool has_interface(
        const std::vector<hardware_interface::InterfaceInfo> &interfaces,
        const char *interface_name)
    {
      return std::any_of(
          interfaces.begin(), interfaces.end(),
          [interface_name](const hardware_interface::InterfaceInfo &interface_info)
          {
            return interface_info.name == interface_name;
          });
    }

    std::string to_lower(std::string value)
    {
      std::transform(
          value.begin(), value.end(), value.begin(),
          [](unsigned char character)
          {
            return static_cast<char>(std::tolower(character));
          });
      return value;
    }

    bool parse_bool_parameter(const std::string &value)
    {
      const auto normalized_value = to_lower(value);
      return normalized_value == "true" ||
             normalized_value == "1" ||
             normalized_value == "yes" ||
             normalized_value == "on";
    }

    bool parse_int_parameter(const std::string &value, int &output)
    {
      try
      {
        output = std::stoi(value);
        return true;
      }
      catch (const std::exception &)
      {
        return false;
      }
    }

    bool parse_double_parameter(const std::string &value, double &output)
    {
      try
      {
        output = std::stod(value);
        return true;
      }
      catch (const std::exception &)
      {
        return false;
      }
    }

    std::vector<std::string> parse_string_list_parameter(const std::string &raw_value)
    {
      std::vector<std::string> values;
      std::stringstream value_stream(raw_value);
      std::string item;
      while (std::getline(value_stream, item, ','))
      {
        item.erase(
            item.begin(),
            std::find_if(item.begin(), item.end(), [](unsigned char character)
                         { return !std::isspace(character); }));
        item.erase(
            std::find_if(item.rbegin(), item.rend(), [](unsigned char character)
                         { return !std::isspace(character); })
                .base(),
            item.end());
        if (!item.empty())
        {
          values.push_back(item);
        }
      }
      return values;
    }

    std::string read_joint_type_parameter(const hardware_interface::ComponentInfo &joint)
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
        const hardware_interface::ComponentInfo &joint,
        const char *parameter_name,
        NumericT &output);

    template <>
    bool read_joint_parameter<int>(
        const hardware_interface::ComponentInfo &joint,
        const char *parameter_name,
        int &output)
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
        const hardware_interface::ComponentInfo &joint,
        const char *parameter_name,
        double &output)
    {
      const auto parameter_it = joint.parameters.find(parameter_name);
      if (parameter_it == joint.parameters.end())
      {
        return false;
      }
      return parse_double_parameter(parameter_it->second, output);
    }

  } // namespace

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
      const hardware_interface::HardwareInfo &info)
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
      const auto &joint = info_.joints[joint_index];
      joint_names_[joint_index] = joint.name;

      for (const auto *interface_name : kInterfaceNames)
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
    use_mock_mode_ =
        use_mock_parameter != info_.hardware_parameters.end() &&
        parse_bool_parameter(use_mock_parameter->second);

    const auto startup_enabled_joints_parameter = info_.hardware_parameters.find("startup_enabled_joints");
    if (startup_enabled_joints_parameter != info_.hardware_parameters.end())
    {
      startup_enabled_joints_ = parse_string_list_parameter(startup_enabled_joints_parameter->second);
    }
    if (startup_enabled_joints_.empty())
    {
      startup_enabled_joints_.push_back("pitch_joint");
    }

    if (use_mock_mode_)
    {
      RCLCPP_WARN(logger_, "Hardware parameter 'use_mock' is enabled; using MockDriver instead of physical CAN hardware");
      driver_ = std::make_unique<MockDriver>(kJointCount);
    }
    else
    {
      RCLCPP_INFO(logger_, "Hardware parameter 'use_mock' is disabled; using physical CANDriver");
      int can_channel = 0;
      const auto can_channel_parameter = info_.hardware_parameters.find("can_channel");
      if (can_channel_parameter != info_.hardware_parameters.end() &&
          !parse_int_parameter(can_channel_parameter->second, can_channel))
      {
        RCLCPP_ERROR(logger_, "Invalid hardware parameter 'can_channel': '%s'", can_channel_parameter->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (can_channel < 0 || can_channel > 1)
      {
        RCLCPP_ERROR(logger_, "Hardware parameter 'can_channel' must be 0 or 1, got %d", can_channel);
        return hardware_interface::CallbackReturn::ERROR;
      }

      int baud_rate = 500;
      const auto baud_rate_parameter = info_.hardware_parameters.find("baud_rate");
      if (baud_rate_parameter != info_.hardware_parameters.end() &&
          !parse_int_parameter(baud_rate_parameter->second, baud_rate))
      {
        RCLCPP_ERROR(logger_, "Invalid hardware parameter 'baud_rate': '%s'", baud_rate_parameter->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      std::vector<CANDriver::JointConfig> joint_configs;
      joint_configs.reserve(kJointCount);
      for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
      {
        const auto &joint = info_.joints[joint_index];
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
      const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    RCLCPP_INFO(logger_, "Configuring Motor Control Hardware Interface");
    if (use_mock_mode_)
    {
      RCLCPP_INFO(logger_, "Mock mode active: skipping physical hardware open/configure steps");
      start_command_listener();
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    if (!driver_->open())
    {
      RCLCPP_ERROR(logger_, "Failed to open motor driver");
      return hardware_interface::CallbackReturn::ERROR;
    }
    start_command_listener();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MotorControlHardwareInterface::on_activate(
      const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    RCLCPP_INFO(logger_, "Activating Motor Control Hardware Interface");
    if (read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.0)) != hardware_interface::return_type::OK)
    {
      RCLCPP_ERROR(logger_, "Failed to read current state before activation");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!sync_commands_to_current_state())
    {
      RCLCPP_ERROR(logger_, "Failed to sync commands to current joint state before activation");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (use_mock_mode_)
    {
      RCLCPP_INFO(logger_, "Mock mode active: skipping joint enable and hardware activation steps");
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::lock_guard<std::mutex> driver_lock(driver_mutex_);
    for (const auto &joint_name : startup_enabled_joints_)
    {
      if (!driver_->enable_joint(joint_name))
      {
        RCLCPP_ERROR(logger_, "Failed to enable startup joint '%s'", joint_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (write(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.0)) != hardware_interface::return_type::OK)
    {
      RCLCPP_ERROR(logger_, "Failed to write hold-position command after startup enable");
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MotorControlHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    RCLCPP_INFO(logger_, "Deactivating Motor Control Hardware Interface");
    stop_command_listener();
    if (use_mock_mode_)
    {
      RCLCPP_INFO(logger_, "Mock mode active: skipping physical hardware disable steps");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::lock_guard<std::mutex> driver_lock(driver_mutex_);
    if (!driver_->disable())
    {
      RCLCPP_ERROR(logger_, "Failed to disable motor driver");
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MotorControlHardwareInterface::on_shutdown(
      const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    RCLCPP_INFO(logger_, "Shutting down Motor Control Hardware Interface");
    stop_command_listener();
    if (driver_)
    {
      std::lock_guard<std::mutex> driver_lock(driver_mutex_);
      if (!use_mock_mode_)
      {
        driver_->disable();
        driver_->close();
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MotorControlHardwareInterface::on_error(
      const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    RCLCPP_ERROR(logger_, "Motor Control Hardware Interface error detected");
    stop_command_listener();
    if (driver_)
    {
      std::lock_guard<std::mutex> driver_lock(driver_mutex_);
      if (!use_mock_mode_)
      {
        driver_->disable();
        driver_->close();
      }
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
      const auto &joint_name = joint_names_[joint_index];
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
      const auto &joint_name = joint_names_[joint_index];
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
      const rclcpp::Time &time, const rclcpp::Duration &period)
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
      const rclcpp::Time &time, const rclcpp::Duration &period)
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

  bool MotorControlHardwareInterface::sync_commands_to_current_state()
  {
    if (state_pos_.size() != cmd_pos_.size() ||
        state_vel_.size() != cmd_vel_.size() ||
        state_eff_.size() != cmd_eff_.size())
    {
      return false;
    }

    cmd_pos_ = state_pos_;
    std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
    std::fill(cmd_eff_.begin(), cmd_eff_.end(), 0.0);
    return true;
  }

  void MotorControlHardwareInterface::start_command_listener()
  {
    if (command_thread_running_)
    {
      return;
    }

    command_node_ = std::make_shared<rclcpp::Node>("motor_axis_command_listener");
    axis_command_sub_ = command_node_->create_subscription<std_msgs::msg::String>(
        "/motor_control/axis_command",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
          handle_axis_command(msg);
        });

    command_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    command_executor_->add_node(command_node_);
    command_thread_running_ = true;
    command_spin_thread_ = std::thread([this]()
                                       { command_executor_->spin(); });
  }

  void MotorControlHardwareInterface::stop_command_listener()
  {
    if (!command_thread_running_)
    {
      return;
    }

    command_thread_running_ = false;
    if (command_executor_)
    {
      command_executor_->cancel();
    }
    if (command_spin_thread_.joinable())
    {
      command_spin_thread_.join();
    }
    if (command_executor_ && command_node_)
    {
      command_executor_->remove_node(command_node_);
    }
    axis_command_sub_.reset();
    command_executor_.reset();
    command_node_.reset();
  }

  void MotorControlHardwareInterface::handle_axis_command(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    std::istringstream command_stream(msg->data);
    std::string action;
    std::string joint_name;
    command_stream >> action >> joint_name;
    if (action.empty() || joint_name.empty())
    {
      RCLCPP_WARN(logger_, "Ignoring axis command '%s'. Expected 'enable <joint>' or 'disable <joint>'", msg->data.c_str());
      return;
    }

    if (!sync_commands_to_current_state())
    {
      RCLCPP_WARN(logger_, "Ignoring axis command '%s' because command/state buffers are not aligned", msg->data.c_str());
      return;
    }

    std::lock_guard<std::mutex> driver_lock(driver_mutex_);
    if (use_mock_mode_)
    {
      RCLCPP_INFO(logger_, "Mock mode active: ignoring axis command '%s %s'", action.c_str(), joint_name.c_str());
      return;
    }
    if (action == "enable")
    {
      if (!driver_->enable_joint(joint_name))
      {
        RCLCPP_ERROR(logger_, "Failed to enable joint '%s' from axis command", joint_name.c_str());
        return;
      }
      if (!driver_->write(cmd_pos_, cmd_vel_, cmd_eff_))
      {
        RCLCPP_ERROR(logger_, "Enabled joint '%s' but failed to send hold-position command", joint_name.c_str());
        return;
      }
      RCLCPP_INFO(logger_, "Enabled joint '%s' and sent hold-position command", joint_name.c_str());
      return;
    }

    if (action == "disable")
    {
      if (!driver_->disable_joint(joint_name))
      {
        RCLCPP_ERROR(logger_, "Failed to disable joint '%s' from axis command", joint_name.c_str());
        return;
      }
      RCLCPP_INFO(logger_, "Disabled joint '%s'", joint_name.c_str());
      return;
    }

    RCLCPP_WARN(logger_, "Ignoring unknown axis command action '%s'", action.c_str());
  }

} // namespace motor_control

PLUGINLIB_EXPORT_CLASS(motor_control::MotorControlHardwareInterface,
                       hardware_interface::SystemInterface)
