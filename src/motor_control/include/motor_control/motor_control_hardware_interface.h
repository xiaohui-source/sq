#ifndef MOTOR_CONTROL_HARDWARE_INTERFACE_H_
#define MOTOR_CONTROL_HARDWARE_INTERFACE_H_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace motor_control
{

class IMotorDriver;

class MotorControlHardwareInterface : public hardware_interface::SystemInterface
{
public:
  MotorControlHardwareInterface();
  virtual ~MotorControlHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  static constexpr std::size_t kJointCount = 3;
  static constexpr std::size_t kInterfaceCount = 3;
  static constexpr std::size_t kPositionInterface = 0;
  static constexpr std::size_t kVelocityInterface = 1;
  static constexpr std::size_t kEffortInterface = 2;

  // Joint information
  std::array<std::string, kJointCount> joint_names_;
  
  // Exported ros2_control interfaces store raw pointers into these vectors.
  // They are sized once during construction and must not be resized after init.
  std::vector<double> state_pos_;
  std::vector<double> state_vel_;
  std::vector<double> state_eff_;
  std::vector<double> cmd_pos_;
  std::vector<double> cmd_vel_;
  std::vector<double> cmd_eff_;

  std::unique_ptr<IMotorDriver> driver_;

  // Logger
  rclcpp::Logger logger_;
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_HARDWARE_INTERFACE_H_
