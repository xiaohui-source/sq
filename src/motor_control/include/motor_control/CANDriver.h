#ifndef MOTOR_CONTROL_CAN_DRIVER_H_
#define MOTOR_CONTROL_CAN_DRIVER_H_

#include "motor_control/IMotorDriver.h"
#include "motor_control/controlcan.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace motor_control
{

class CANDriver : public IMotorDriver
{
public:
  struct JointConfig
  {
    std::string name;
    std::uint32_t motor_id = 0;
    double gear_ratio = 1.0;
    double force_max = 0.0;
    bool is_revolute = true;
    double position_min = 0.0;
    double position_max = 0.0;
    double output_translation_per_rev = 0.0;
  };

  CANDriver(std::vector<JointConfig> joint_configs, std::uint32_t can_channel, std::uint32_t baud_rate);
  ~CANDriver() override;

  bool init() override;
  bool open() override;
  bool enable() override;
  bool disable() override;
  bool enable_joint(const std::string& joint_name) override;
  bool disable_joint(const std::string& joint_name) override;
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
  enum class ControlMode : std::uint8_t
  {
    kPosition = 1,
    kVelocity = 2,
    kEffort = 3,
  };

  struct JointState
  {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
    bool actually_enabled = false;
    bool has_fault = false;
  };

  bool transmit_frame(const VCI_CAN_OBJ& frame);
  void encode_mode_frame(std::size_t joint_index, int mode, VCI_CAN_OBJ& frame) const;
  void encode_control_frame(
      std::size_t joint_index,
      ControlMode mode,
      double cmd_pos,
      double cmd_vel,
      double cmd_eff,
      VCI_CAN_OBJ& frame) const;
  void decode_feedback_frame(std::size_t joint_index, const VCI_CAN_OBJ& frame);
  bool read_nonblocking_feedback();
  std::size_t find_joint_index_by_frame_id(std::uint32_t frame_id) const;
  ControlMode choose_mode(std::size_t joint_index, double cmd_pos, double cmd_vel, double cmd_eff) const;
  bool all_zero_references_initialized() const;
  bool is_joint_enabled(std::size_t joint_index) const;
  void capture_zero_reference_if_needed(std::size_t joint_index, double absolute_position);
  double clamp_gear_ratio(double gear_ratio) const;
  double clamp_command_position(std::size_t joint_index, double position) const;
  std::size_t find_joint_index_by_name(const std::string& joint_name) const;
  bool set_joint_enabled(std::size_t joint_index, bool enabled);
  std::uint32_t baud_to_timing0() const;
  std::uint32_t baud_to_timing1() const;

  const std::vector<JointConfig> joint_configs_;
  const std::uint32_t can_channel_;
  const std::uint32_t baud_rate_;

  mutable std::mutex mutex_;
  bool initialized_ = false;
  bool device_handle_open_ = false;
  bool device_open_ = false;
  bool enabled_ = false;

  std::vector<JointState> joint_states_;
  std::vector<double> last_cmd_pos_;
  std::vector<double> last_cmd_vel_;
  std::vector<double> last_cmd_eff_;
  std::vector<ControlMode> last_modes_;
  std::vector<bool> mode_initialized_;
  std::vector<double> zero_reference_pos_;
  std::vector<bool> zero_reference_initialized_;
  std::vector<bool> joint_enabled_;
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_CAN_DRIVER_H_
