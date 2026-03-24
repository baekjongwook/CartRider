// Hardware Node
// 2026.03.24 백종욱

#include "cartrider_vesc_sdk/hardware_node.hpp"

#include <net/if.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <cmath>

using cartrider_vesc_sdk::msg::MotorCommandArray;
using cartrider_vesc_sdk::msg::MotorState;
using cartrider_vesc_sdk::msg::MotorStateArray;

RawCanSocket::RawCanSocket(const std::string& ifname)
{
  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0)
    throw std::runtime_error("Failed to open CAN socket");

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
  {
    ::close(socket_);
    throw std::runtime_error("Failed to get CAN interface index");
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    ::close(socket_);
    throw std::runtime_error("Failed to bind CAN socket");
  }

  struct timeval tv {};
  tv.tv_sec = 0;
  tv.tv_usec = 10000;
  if (::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
  {
    ::close(socket_);
    throw std::runtime_error("Failed to set CAN receive timeout");
  }
}

RawCanSocket::~RawCanSocket()
{
  if (socket_ >= 0)
    ::close(socket_);
}

RawCanSocket::RawCanSocket(RawCanSocket&& other) noexcept
: socket_(other.socket_)
{
  other.socket_ = -1;
}

RawCanSocket& RawCanSocket::operator=(RawCanSocket&& other) noexcept
{
  if (this != &other)
  {
    if (socket_ >= 0)
      ::close(socket_);
    socket_ = other.socket_;
    other.socket_ = -1;
  }
  return *this;
}

bool RawCanSocket::writeFrame(const can_frame& frame) const
{
  return ::write(socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool RawCanSocket::readFrame(can_frame& frame) const
{
  return ::read(socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

int RawCanSocket::fd() const noexcept
{
  return socket_;
}

HardwareNode::HardwareNode()
: rclcpp::Node("hardware_node")
{
  RCLCPP_INFO(this->get_logger(), "Hardware Node Started");

  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("command_timeout_ms", 2000);

  this->declare_parameter("motor_ids", std::vector<int64_t>{});
  this->declare_parameter("operate_modes", std::vector<std::string>{});

  this->declare_parameter("current_min", std::vector<double>{});
  this->declare_parameter("current_max", std::vector<double>{});

  this->declare_parameter("speed_min", std::vector<double>{});
  this->declare_parameter("speed_max", std::vector<double>{});
  this->declare_parameter("speed_deadzone", std::vector<double>{});

  this->declare_parameter("position_min", std::vector<double>{});
  this->declare_parameter("position_max", std::vector<double>{});
  this->declare_parameter("position_omega_max", std::vector<double>{});

  this->declare_parameter("pole_pairs", std::vector<int64_t>{});
  this->declare_parameter("gear_ratios", std::vector<double>{});

  can_interface_ = this->get_parameter("can_interface").as_string();
  command_timeout_ms_ = this->get_parameter("command_timeout_ms").as_int();

  driver_ = std::make_unique<RawCanSocket>(can_interface_);
  createMotorsFromParameters();

  last_command_time_ = this->now();
  last_reconnect_attempt_ = this->now();

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&HardwareNode::controlLoop, this));

  state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&HardwareNode::stateLoop, this));

  subscription_ = this->create_subscription<MotorCommandArray>(
    "vesc_command",
    10,
    std::bind(&HardwareNode::commandCallback, this, std::placeholders::_1));

  publication_ = this->create_publisher<MotorStateArray>("vesc_state", 10);
}

HardwareNode::~HardwareNode()
{
  timer_->cancel();
  state_timer_->cancel();

  std::lock_guard<std::mutex> lock(mutex_);
  shutdownAllMotors();
}

void HardwareNode::shutdownAllMotors()
{
  for (auto& m : motors_)
  {
    try
    {
      switch (m.mode)
      {
        case ControlMode::CURRENT:
          sendCurrentCommand(m.id, 0.0);
          break;

        case ControlMode::SPEED:
          sendSpeedCommand(m, 0.0);
          break;

        case ControlMode::POSITION:
          sendBrakeCommand(m.id, 0.0);
          break;
      }

      m.stopped = true;
    }
    catch (...)
    {
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void HardwareNode::createMotorsFromParameters()
{
  motors_.clear();

  std::vector<int64_t> motor_ids;
  std::vector<std::string> operate_modes;
  std::vector<double> current_min, current_max;
  std::vector<double> speed_min, speed_max, speed_deadzone;
  std::vector<double> position_min, position_max, position_omega_max;
  std::vector<int64_t> pole_pairs;
  std::vector<double> gear_ratios;

  try
  {
    this->get_parameter("motor_ids", motor_ids);
    this->get_parameter("operate_modes", operate_modes);

    this->get_parameter("current_min", current_min);
    this->get_parameter("current_max", current_max);

    this->get_parameter("speed_min", speed_min);
    this->get_parameter("speed_max", speed_max);
    this->get_parameter("speed_deadzone", speed_deadzone);

    this->get_parameter("position_min", position_min);
    this->get_parameter("position_max", position_max);
    this->get_parameter("position_omega_max", position_omega_max);

    this->get_parameter("pole_pairs", pole_pairs);
    this->get_parameter("gear_ratios", gear_ratios);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter loading failed: %s", e.what());
    return;
  }

  const std::size_t n = motor_ids.size();

  if (n == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "No motors defined!");
    return;
  }

  if (operate_modes.size() != n)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter size mismatch: operate_modes");
    return;
  }

  auto check_size_double = [&](const std::vector<double>& v, const std::string& name)
  {
    if (v.size() != n)
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter size mismatch: %s", name.c_str());
      return false;
    }
    return true;
  };

  auto check_size_int = [&](const std::vector<int64_t>& v, const std::string& name)
  {
    if (v.size() != n)
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter size mismatch: %s", name.c_str());
      return false;
    }
    return true;
  };

  if (!check_size_double(current_min, "current_min") ||
      !check_size_double(current_max, "current_max") ||
      !check_size_double(speed_min, "speed_min") ||
      !check_size_double(speed_max, "speed_max") ||
      !check_size_double(speed_deadzone, "speed_deadzone") ||
      !check_size_double(position_min, "position_min") ||
      !check_size_double(position_max, "position_max") ||
      !check_size_double(position_omega_max, "position_omega_max") ||
      !check_size_int(pole_pairs, "pole_pairs") ||
      !check_size_double(gear_ratios, "gear_ratios"))
  {
    return;
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    MotorUnit unit;
    unit.id = static_cast<int>(motor_ids[i]);

    if (operate_modes[i] == "current")
      unit.mode = ControlMode::CURRENT;
    else if (operate_modes[i] == "speed")
      unit.mode = ControlMode::SPEED;
    else if (operate_modes[i] == "position")
      unit.mode = ControlMode::POSITION;
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid operate_mode: %s", operate_modes[i].c_str());
      return;
    }

    unit.current_min = current_min[i];
    unit.current_max = current_max[i];

    unit.speed_min = speed_min[i];
    unit.speed_max = speed_max[i];
    unit.speed_deadzone = speed_deadzone[i];

    unit.position_min = position_min[i];
    unit.position_max = position_max[i];
    unit.position_omega_max = position_omega_max[i];

    unit.pole_pairs = static_cast<int>(pole_pairs[i]);
    unit.gear_ratio = gear_ratios[i];

    unit.target = 0.0;
    unit.stopped = true;

    motors_.push_back(unit);

    RCLCPP_INFO(
      this->get_logger(),
      "Motor %d initialized | mode=%s | pole_pairs=%d | gear_ratio=%.3f",
      unit.id,
      operate_modes[i].c_str(),
      unit.pole_pairs,
      unit.gear_ratio
    );
  }

  RCLCPP_INFO(this->get_logger(), "Initialized %zu motors", motors_.size());
  RCLCPP_WARN(this->get_logger(), "Position control is currently disabled because encoder is not installed");
}

void HardwareNode::controlLoop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!can_ok_)
  {
    attemptReconnect();
    return;
  }

  if ((this->now() - last_command_time_).nanoseconds() / 1000000 > command_timeout_ms_)
  {
    shutdownAllMotors();
    return;
  }

  try
  {
    for (auto& m : motors_)
    {
      switch (m.mode)
      {
        case ControlMode::CURRENT:
          currentControl(m);
          break;

        case ControlMode::SPEED:
          speedControl(m);
          break;

        case ControlMode::POSITION:
          positionControl(m);
          break;
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "CAN fatal error: %s", e.what());

    backup_targets_.clear();
    for (auto& m : motors_)
    {
      backup_targets_.push_back({m.mode, m.target});
      try
      {
        switch (m.mode)
        {
          case ControlMode::CURRENT:
            sendCurrentCommand(m.id, 0.0);
            break;

          case ControlMode::SPEED:
            sendSpeedCommand(m, 0.0);
            break;

          case ControlMode::POSITION:
            sendBrakeCommand(m.id, 0.0);
            break;
        }
      }
      catch (...)
      {
      }
    }

    can_ok_ = false;
    last_reconnect_attempt_ = this->now();
  }
}

void HardwareNode::stateLoop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!can_ok_)
    return;

  MotorStateArray state_msg;
  state_msg.stamp = this->now();

  try
  {
    for (auto& m : motors_)
    {
      MotorState s;
      s.id = m.id;

      // 아직 VESC 상태 read 미구현
      s.current = 0.0;
      s.speed = 0.0;
      s.position = 0.0;

      state_msg.states.push_back(s);
    }

    publication_->publish(state_msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "CAN fatal error: %s", e.what());
    can_ok_ = false;
    last_reconnect_attempt_ = this->now();
  }
}

void HardwareNode::attemptReconnect()
{
  if ((this->now() - last_reconnect_attempt_).seconds() < 1.0)
    return;

  RCLCPP_WARN(this->get_logger(), "Attempting CAN reconnect...");

  try
  {
    driver_.reset();
    driver_ = std::make_unique<RawCanSocket>(can_interface_);
    createMotorsFromParameters();

    if (backup_targets_.size() == motors_.size())
    {
      for (std::size_t i = 0; i < motors_.size(); ++i)
      {
        motors_[i].target = backup_targets_[i].second;
        motors_[i].stopped = false;
      }
    }

    can_ok_ = true;
    last_reconnect_attempt_ = this->now();
    last_command_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "CAN Reconnected Successfully");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Reconnect failed: %s", e.what());
    last_reconnect_attempt_ = this->now();
  }
}

void HardwareNode::currentControl(MotorUnit& m)
{
  const double target_current = std::clamp(m.target, m.current_min, m.current_max);

  if (!sendCurrentCommand(m.id, target_current))
    throw std::runtime_error("sendCurrentCommand failed");

  m.stopped = false;
}

void HardwareNode::speedControl(MotorUnit& m)
{
  double target_rpm = std::clamp(m.target, m.speed_min, m.speed_max);

  if (std::abs(target_rpm) < m.speed_deadzone)
  {
    if (!m.stopped)
    {
      if (!sendSpeedCommand(m, 0.0))
        throw std::runtime_error("sendSpeedCommand(0) failed");
      m.stopped = true;
    }
    return;
  }

  if (!sendSpeedCommand(m, target_rpm))
    throw std::runtime_error("sendSpeedCommand failed");

  m.stopped = false;
}

void HardwareNode::positionControl(MotorUnit& m)
{
  (void)m;
  // 엔코더 없어서 현재 비활성
}

void HardwareNode::commandCallback(const MotorCommandArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->commands.size() != motors_.size())
  {
    RCLCPP_WARN(this->get_logger(), "Command size mismatch: expected %zu, got %zu", motors_.size(), msg->commands.size());
    return;
  }

  for (const auto& cmd : msg->commands)
  {
    auto it = std::find_if(
      motors_.begin(),
      motors_.end(),
      [&](const MotorUnit& m){ return m.id == cmd.id; });

    if (it == motors_.end())
    {
      RCLCPP_WARN(this->get_logger(), "Unknown motor id in command: %d", cmd.id);
      return;
    }
  }

  last_command_time_ = this->now();

  for (const auto& cmd : msg->commands)
  {
    auto it = std::find_if(
      motors_.begin(),
      motors_.end(),
      [&](const MotorUnit& m){ return m.id == cmd.id; });

    it->target = cmd.target;
  }
}

bool HardwareNode::sendCurrentCommand(int id, double current_a)
{
  const std::int32_t value = static_cast<std::int32_t>(current_a * 1000.0);

  can_frame frame {};
  frame.can_id = ((1 << 8) | id) | CAN_EFF_FLAG;
  frame.can_dlc = 4;

  frame.data[0] = (value >> 24) & 0xFF;
  frame.data[1] = (value >> 16) & 0xFF;
  frame.data[2] = (value >> 8) & 0xFF;
  frame.data[3] = value & 0xFF;

  if (!driver_->writeFrame(frame))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send current command to motor %d", id);
    return false;
  }

  return true;
}

bool HardwareNode::sendSpeedCommand(const MotorUnit& m, double speed_rpm)
{
  const double erpm_d = speed_rpm * m.gear_ratio * static_cast<double>(m.pole_pairs);
  const std::int32_t erpm = static_cast<std::int32_t>(erpm_d);

  can_frame frame {};
  frame.can_id = ((3 << 8) | m.id) | CAN_EFF_FLAG;
  frame.can_dlc = 4;

  frame.data[0] = (erpm >> 24) & 0xFF;
  frame.data[1] = (erpm >> 16) & 0xFF;
  frame.data[2] = (erpm >> 8) & 0xFF;
  frame.data[3] = erpm & 0xFF;

  if (!driver_->writeFrame(frame))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send speed command to motor %d", m.id);
    return false;
  }

  return true;
}

bool HardwareNode::sendPositionCommand(int id, double position_deg)
{
  (void)id;
  (void)position_deg;
  return false;
}

bool HardwareNode::sendBrakeCommand(int id, double brake_current_a)
{
  const std::int32_t value = static_cast<std::int32_t>(brake_current_a * 1000.0);

  can_frame frame {};
  frame.can_id = ((2 << 8) | id) | CAN_EFF_FLAG;
  frame.can_dlc = 4;

  frame.data[0] = (value >> 24) & 0xFF;
  frame.data[1] = (value >> 16) & 0xFF;
  frame.data[2] = (value >> 8) & 0xFF;
  frame.data[3] = value & 0xFF;

  if (!driver_->writeFrame(frame))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send brake command to motor %d", id);
    return false;
  }

  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}