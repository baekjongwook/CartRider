#pragma once

#include <linux/can.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "cartrider_vesc_sdk/msg/motor_command_array.hpp"
#include "cartrider_vesc_sdk/msg/motor_state.hpp"
#include "cartrider_vesc_sdk/msg/motor_state_array.hpp"

class RawCanSocket
{
public:
  explicit RawCanSocket(const std::string &ifname);
  ~RawCanSocket();

  RawCanSocket(const RawCanSocket &) = delete;
  RawCanSocket &operator=(const RawCanSocket &) = delete;
  RawCanSocket(RawCanSocket &&other) noexcept;
  RawCanSocket &operator=(RawCanSocket &&other) noexcept;

  bool writeFrame(const can_frame &frame) const;
  bool readFrame(can_frame &frame) const;
  int fd() const noexcept;

private:
  int socket_{-1};
};

enum class ControlMode
{
  CURRENT,
  SPEED,
  POSITION
};

struct VescStatusCache
{
  bool status1_valid{false};
  bool status4_valid{false};

  double motor_current_a{0.0};
  double input_current_a{0.0};
  double duty{0.0};
  double erpm{0.0};

  double speed_radps_out{0.0};

  double position_rad{0.0};

  double temp_fet_c{0.0};
  double temp_motor_c{0.0};

  rclcpp::Time last_update;
};

struct MotorUnit
{
  int id{0};
  ControlMode mode{ControlMode::CURRENT};

  double current_min{0.0};
  double current_max{0.0};

  double speed_min{0.0};
  double speed_max{0.0};
  double speed_deadzone{0.0};

  double position_min{0.0};
  double position_max{0.0};
  double position_zero_offset{0.0};

  int pole_pairs{0};
  double gear_ratio{1.0};

  double target{0.0};

  bool stopped{true};

  VescStatusCache state;
};

class HardwareNode : public rclcpp::Node
{
public:
  HardwareNode();
  ~HardwareNode();

private:
  void createMotorsFromParameters();
  void controlLoop();
  void stateLoop();
  void commandCallback(
      const cartrider_vesc_sdk::msg::MotorCommandArray::SharedPtr msg);
  void shutdownAllMotors();
  void attemptReconnect();

  void currentControl(MotorUnit &m);
  void speedControl(MotorUnit &m);
  void positionControl(MotorUnit &m);

  bool sendCurrentCommand(int id, double current_a);
  bool sendSpeedCommandRadps(const MotorUnit &m, double target_speed_radps);
  bool sendPositionCommandDegrees(int id, double target_position_deg);
  bool sendBrakeCommand(int id, double brake_current_a);

  void pumpCanRx(std::size_t max_frames = 100);
  void parseCanFrame(const can_frame &frame);
  MotorUnit *findMotorById(int id);

  static double wrapDegrees360(double degrees);
  static double wrapDegrees180Signed(double degrees);

  double softwarePositionRadToRawPositionDeg(
      const MotorUnit &motor,
      double software_position_rad);

  double rawPositionDegToSoftwarePositionRad(
      const MotorUnit &motor,
      double raw_position_deg);

  static int16_t readInt16BE(const uint8_t *data);
  static int32_t readInt32BE(const uint8_t *data);

private:
  std::string can_interface_;
  int command_timeout_ms_{2000};

  std::unique_ptr<RawCanSocket> driver_;
  std::vector<MotorUnit> motors_;
  std::vector<std::pair<ControlMode, double>> backup_targets_;

  bool can_ok_{true};
  rclcpp::Time last_command_time_;
  rclcpp::Time last_reconnect_attempt_;

  std::mutex mutex_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  rclcpp::Subscription<cartrider_vesc_sdk::msg::MotorCommandArray>::SharedPtr subscription_;
  rclcpp::Publisher<cartrider_vesc_sdk::msg::MotorStateArray>::SharedPtr publication_;
};