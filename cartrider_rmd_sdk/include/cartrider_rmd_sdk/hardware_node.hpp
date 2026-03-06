// Hardware Node
// 2026.02.18 백종욱

#ifndef CARTRIDER_RMD_SDK_HARDWARE_NODE__HPP_
#define CARTRIDER_RMD_SDK_HARDWARE_NODE__HPP_

#include <rclcpp/rclcpp.hpp>

#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"
#include "cartrider_rmd_sdk/msg/motor_state_array.hpp"

#include <myactuator_rmd/myactuator_rmd.hpp>
#include <myactuator_rmd/can/exceptions.hpp>

#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <utility>

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
    int id;
    std::unique_ptr<myactuator_rmd::ActuatorInterface> motor;

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
  void commandCallback(const cartrider_rmd_sdk::msg::MotorCommandArray::SharedPtr msg);

  std::unique_ptr<myactuator_rmd::CanDriver> driver_;
  std::vector<MotorUnit> motors_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  rclcpp::Subscription<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr subscription_;
  rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorStateArray>::SharedPtr publication_;

  std::mutex mutex_;

  std::atomic<bool> can_ok_{true};
  std::string can_interface_;
  
  rclcpp::Time last_command_time_;    
  int command_timeout_ms_{2000};      

  rclcpp::Time last_reconnect_attempt_; 
  std::vector<std::pair<ControlMode, double>> backup_targets_;
};

#endif