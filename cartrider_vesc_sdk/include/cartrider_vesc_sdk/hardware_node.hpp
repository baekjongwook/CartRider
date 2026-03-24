// Hardware Node
// 2026.03.24 백종욱

#ifndef HARDWARE_NODE_HPP
#define HARDWARE_NODE_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "cartrider_vesc_sdk/msg/motor_command_array.hpp"
#include "cartrider_vesc_sdk/msg/motor_state_array.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <utility>
#include <cstdint>

class RawCanSocket
{
public:
  explicit RawCanSocket(const std::string& ifname);
  ~RawCanSocket();

  RawCanSocket(const RawCanSocket&) = delete;
  RawCanSocket& operator=(const RawCanSocket&) = delete;

  RawCanSocket(RawCanSocket&& other) noexcept;
  RawCanSocket& operator=(RawCanSocket&& other) noexcept;

  bool writeFrame(const can_frame& frame) const;
  bool readFrame(can_frame& frame) const;
  int fd() const noexcept;

private:
  int socket_{-1};
};

class HardwareNode : public rclcpp::Node
{
public:
  HardwareNode();
  ~HardwareNode();

private:
  enum class ControlMode
  {
    CURRENT,
    SPEED,
    POSITION
  };

  struct MotorUnit
  {
    int id{0};
    double target{0.0};
    bool stopped{true};

    double current_min{0.0};
    double current_max{0.0};

    double speed_min{0.0};
    double speed_max{0.0};
    double speed_deadzone{0.0};

    double position_min{0.0};
    double position_max{0.0};
    double position_omega_max{0.0};

    int pole_pairs{1};
    double gear_ratio{1.0};

    ControlMode mode{ControlMode::SPEED};
  };

  void shutdownAllMotors();
  void attemptReconnect();
  void createMotorsFromParameters();

  void controlLoop();
  void stateLoop();

  void currentControl(MotorUnit& m);
  void speedControl(MotorUnit& m);
  void positionControl(MotorUnit& m);

  void commandCallback(const cartrider_vesc_sdk::msg::MotorCommandArray::SharedPtr msg);

  bool sendCurrentCommand(int id, double current_a);
  bool sendSpeedCommand(const MotorUnit& m, double speed_rpm);
  bool sendPositionCommand(int id, double position_deg);
  bool sendBrakeCommand(int id, double brake_current_a);

  std::unique_ptr<RawCanSocket> driver_;
  std::vector<MotorUnit> motors_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  rclcpp::Subscription<cartrider_vesc_sdk::msg::MotorCommandArray>::SharedPtr subscription_;
  rclcpp::Publisher<cartrider_vesc_sdk::msg::MotorStateArray>::SharedPtr publication_;

  std::mutex mutex_;
  std::atomic<bool> can_ok_{true};

  std::string can_interface_;
  rclcpp::Time last_command_time_;
  int command_timeout_ms_{2000};
  rclcpp::Time last_reconnect_attempt_;

  std::vector<std::pair<ControlMode, double>> backup_targets_;
};

#endif // HARDWARE_NODE_HPP