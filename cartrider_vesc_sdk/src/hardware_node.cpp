// Hardware Node
// 2026.03.24 백종욱

#include "cartrider_vesc_sdk/hardware_node.hpp"

#include <net/if.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <cstdint>

using cartrider_vesc_sdk::msg::MotorCommandArray;
using cartrider_vesc_sdk::msg::MotorState;
using cartrider_vesc_sdk::msg::MotorStateArray;

namespace
{
  constexpr uint8_t CAN_PACKET_SET_CURRENT = 1;
  constexpr uint8_t CAN_PACKET_SET_CURRENT_BRAKE = 2;
  constexpr uint8_t CAN_PACKET_SET_RPM = 3;
  constexpr uint8_t CAN_PACKET_SET_POS = 4;

  constexpr uint8_t CAN_PACKET_STATUS = 9;
  constexpr uint8_t CAN_PACKET_STATUS_2 = 14;
  constexpr uint8_t CAN_PACKET_STATUS_3 = 15;
  constexpr uint8_t CAN_PACKET_STATUS_4 = 16;

  constexpr double PI = 3.14159265358979323846;
  constexpr double DEG2RAD = PI / 180.0;
  constexpr double RAD2DEG = 180.0 / PI;
  constexpr double RPM2RADPS = 2.0 * PI / 60.0;
  constexpr double RADPS2RPM = 60.0 / (2.0 * PI);
}

RawCanSocket::RawCanSocket(const std::string &ifname)
{
  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0)
    throw std::runtime_error("Failed to open CAN socket");

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
  {
    ::close(socket_);
    throw std::runtime_error("Failed to get CAN interface index");
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    ::close(socket_);
    throw std::runtime_error("Failed to bind CAN socket");
  }

  struct timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 1000;
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

RawCanSocket::RawCanSocket(RawCanSocket &&other) noexcept
    : socket_(other.socket_)
{
  other.socket_ = -1;
}

RawCanSocket &RawCanSocket::operator=(RawCanSocket &&other) noexcept
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

bool RawCanSocket::writeFrame(const can_frame &frame) const
{
  return ::write(socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool RawCanSocket::readFrame(can_frame &frame) const
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
  this->declare_parameter("position_zero_offset", std::vector<double>{});

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

  RCLCPP_INFO(this->get_logger(),
              "[INIT] can_interface=%s command_timeout_ms=%d motors=%zu",
              can_interface_.c_str(), command_timeout_ms_, motors_.size());
}

HardwareNode::~HardwareNode()
{
  timer_->cancel();
  state_timer_->cancel();

  std::lock_guard<std::mutex> lock(mutex_);
  shutdownAllMotors();
}

double HardwareNode::wrapDegrees360(double degrees)
{
  double wrapped = std::fmod(degrees, 360.0);
  if (wrapped < 0.0)
    wrapped += 360.0;
  return wrapped;
}

double HardwareNode::wrapDegrees180Signed(double degrees)
{
  double wrapped = std::fmod(degrees + 180.0, 360.0);
  if (wrapped < 0.0)
    wrapped += 360.0;
  return wrapped - 180.0;
}

int16_t HardwareNode::readInt16BE(const uint8_t *data)
{
  return static_cast<int16_t>(
      (static_cast<uint16_t>(data[0]) << 8) |
      static_cast<uint16_t>(data[1]));
}

int32_t HardwareNode::readInt32BE(const uint8_t *data)
{
  return static_cast<int32_t>(
      (static_cast<uint32_t>(data[0]) << 24) |
      (static_cast<uint32_t>(data[1]) << 16) |
      (static_cast<uint32_t>(data[2]) << 8) |
      static_cast<uint32_t>(data[3]));
}

double HardwareNode::softwarePositionRadToRawPositionDeg(
    const MotorUnit &motor,
    double software_position_rad)
{
  const double software_position_deg = software_position_rad * RAD2DEG;
  return wrapDegrees360(software_position_deg + motor.position_zero_offset);
}

double HardwareNode::rawPositionDegToSoftwarePositionRad(
    const MotorUnit &motor,
    double raw_position_deg)
{
  const double software_position_deg =
      wrapDegrees180Signed(raw_position_deg - motor.position_zero_offset);
  return software_position_deg * DEG2RAD;
}

MotorUnit *HardwareNode::findMotorById(int id)
{
  for (auto &m : motors_)
  {
    if (m.id == id)
      return &m;
  }
  return nullptr;
}

void HardwareNode::shutdownAllMotors()
{
  for (auto &m : motors_)
  {
    try
    {
      switch (m.mode)
      {
      case ControlMode::CURRENT:
        sendCurrentCommand(m.id, 0.0);
        break;

      case ControlMode::SPEED:
        sendSpeedCommandRadps(m, 0.0);
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
  std::vector<double> speed_min_rpm, speed_max_rpm, speed_deadzone_rpm;
  std::vector<double> position_min_deg, position_max_deg, position_zero_offset_deg;
  std::vector<int64_t> pole_pairs;
  std::vector<double> gear_ratios;

  try
  {
    this->get_parameter("motor_ids", motor_ids);
    this->get_parameter("operate_modes", operate_modes);

    this->get_parameter("current_min", current_min);
    this->get_parameter("current_max", current_max);

    this->get_parameter("speed_min", speed_min_rpm);
    this->get_parameter("speed_max", speed_max_rpm);
    this->get_parameter("speed_deadzone", speed_deadzone_rpm);

    this->get_parameter("position_min", position_min_deg);
    this->get_parameter("position_max", position_max_deg);
    this->get_parameter("position_zero_offset", position_zero_offset_deg);

    this->get_parameter("pole_pairs", pole_pairs);
    this->get_parameter("gear_ratios", gear_ratios);
  }
  catch (const std::exception &e)
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

  auto check_size_double = [&](const std::vector<double> &v, const std::string &name)
  {
    if (v.size() != n)
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter size mismatch: %s", name.c_str());
      return false;
    }
    return true;
  };

  auto check_size_int = [&](const std::vector<int64_t> &v, const std::string &name)
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
      !check_size_double(speed_min_rpm, "speed_min") ||
      !check_size_double(speed_max_rpm, "speed_max") ||
      !check_size_double(speed_deadzone_rpm, "speed_deadzone") ||
      !check_size_double(position_min_deg, "position_min") ||
      !check_size_double(position_max_deg, "position_max") ||
      !check_size_double(position_zero_offset_deg, "position_zero_offset") ||
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

    unit.speed_min = speed_min_rpm[i] * RPM2RADPS;
    unit.speed_max = speed_max_rpm[i] * RPM2RADPS;
    unit.speed_deadzone = speed_deadzone_rpm[i] * RPM2RADPS;

    unit.position_min = position_min_deg[i] * DEG2RAD;
    unit.position_max = position_max_deg[i] * DEG2RAD;
    unit.position_zero_offset = position_zero_offset_deg[i];

    unit.pole_pairs = static_cast<int>(pole_pairs[i]);
    unit.gear_ratio = gear_ratios[i];

    unit.target = 0.0;
    unit.stopped = true;

    motors_.push_back(unit);

    RCLCPP_INFO(
        this->get_logger(),
        "[INIT] Motor %d initialized | mode=%s | "
        "current[%.3f, %.3f] A | "
        "speed[%.3f, %.3f] rpm deadzone=%.3f rpm | "
        "pos[%.3f, %.3f] deg zero_offset=%.3f deg | "
        "pole_pairs=%d | gear_ratio=%.3f",
        unit.id,
        operate_modes[i].c_str(),
        current_min[i],
        current_max[i],
        speed_min_rpm[i],
        speed_max_rpm[i],
        speed_deadzone_rpm[i],
        position_min_deg[i],
        position_max_deg[i],
        position_zero_offset_deg[i],
        unit.pole_pairs,
        unit.gear_ratio);
  }

  RCLCPP_INFO(this->get_logger(), "Initialized %zu motors", motors_.size());
}

void HardwareNode::pumpCanRx(std::size_t max_frames)
{
  for (std::size_t i = 0; i < max_frames; ++i)
  {
    can_frame frame{};
    if (!driver_->readFrame(frame))
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        return;

      throw std::runtime_error(std::string("CAN read failed: ") + std::strerror(errno));
    }

    parseCanFrame(frame);
  }
}

void HardwareNode::parseCanFrame(const can_frame &frame)
{
  if ((frame.can_id & CAN_EFF_FLAG) == 0)
    return;

  const uint32_t eid = frame.can_id & CAN_EFF_MASK;
  const uint8_t cmd = static_cast<uint8_t>((eid >> 8) & 0xFF);
  const uint8_t vesc_id = static_cast<uint8_t>(eid & 0xFF);

  MotorUnit *motor = findMotorById(static_cast<int>(vesc_id));
  if (!motor)
    return;

  auto &state = motor->state;
  state.last_update = this->now();

  switch (cmd)
  {
  case CAN_PACKET_STATUS:
  {
    if (frame.can_dlc < 8)
      return;

    const int32_t erpm = readInt32BE(&frame.data[0]);
    const int16_t current_x10 = readInt16BE(&frame.data[4]);
    const int16_t duty_x1000 = readInt16BE(&frame.data[6]);

    state.erpm = static_cast<double>(erpm);
    state.motor_current_a = static_cast<double>(current_x10) / 10.0;
    state.duty = static_cast<double>(duty_x1000) / 1000.0;

    const double speed_conversion_denominator =
        motor->gear_ratio * static_cast<double>(motor->pole_pairs);

    const double output_speed_rpm =
        (std::abs(speed_conversion_denominator) > 1e-9)
            ? (state.erpm / speed_conversion_denominator)
            : 0.0;

    state.speed_radps_out = output_speed_rpm * RPM2RADPS;
    state.status1_valid = true;
    break;
  }

  case CAN_PACKET_STATUS_2:
  case CAN_PACKET_STATUS_3:
    break;

  case CAN_PACKET_STATUS_4:
  {
    if (frame.can_dlc < 8)
      return;

    const int16_t fet_x10 = readInt16BE(&frame.data[0]);
    const int16_t motor_x10 = readInt16BE(&frame.data[2]);
    const int16_t input_current_x10 = readInt16BE(&frame.data[4]);
    const int16_t pid_pos_x50 = readInt16BE(&frame.data[6]);

    state.temp_fet_c = static_cast<double>(fet_x10) / 10.0;
    state.temp_motor_c = static_cast<double>(motor_x10) / 10.0;
    state.input_current_a = static_cast<double>(input_current_x10) / 10.0;

    const double raw_position_deg =
        wrapDegrees360(static_cast<double>(pid_pos_x50) / 50.0);

    const double software_position_rad =
        rawPositionDegToSoftwarePositionRad(*motor, raw_position_deg);

    state.position_rad = std::clamp(
        software_position_rad,
        motor->position_min,
        motor->position_max);

    state.status4_valid = true;
    break;
  }

  default:
    break;
  }
}

void HardwareNode::controlLoop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const double dt_ms =
      (this->now() - last_command_time_).nanoseconds() / 1e6;

  if (!can_ok_)
  {
    attemptReconnect();
    return;
  }

  if (dt_ms > command_timeout_ms_)
  {
    shutdownAllMotors();
    return;
  }

  try
  {
    pumpCanRx(200);

    for (auto &m : motors_)
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
  catch (const std::exception &e)
  {
    const int err = errno;

    RCLCPP_ERROR(this->get_logger(), "[CTRL] CAN error: %s", e.what());

    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)
      return;

    backup_targets_.clear();

    for (auto &m : motors_)
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
          sendSpeedCommandRadps(m, 0.0);
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

    RCLCPP_ERROR(this->get_logger(), "[CTRL] can_ok set false, entering reconnect mode");
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
    pumpCanRx(200);

    for (auto &m : motors_)
    {
      MotorState s;
      s.id = m.id;
      s.current = m.state.motor_current_a;
      s.speed = m.state.speed_radps_out;
      s.position = m.state.position_rad;
      state_msg.states.push_back(s);
    }

    publication_->publish(state_msg);
  }
  catch (const std::exception &e)
  {
    const int err = errno;

    RCLCPP_ERROR(this->get_logger(), "[STATE] CAN error: %s", e.what());

    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)
      return;

    can_ok_ = false;
    last_reconnect_attempt_ = this->now();

    RCLCPP_ERROR(this->get_logger(), "[STATE] can_ok set false");
  }
}

void HardwareNode::attemptReconnect()
{
  if ((this->now() - last_reconnect_attempt_).seconds() < 1.0)
    return;

  RCLCPP_WARN(this->get_logger(), "[RECONNECT] Attempting CAN reconnect...");

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
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "[RECONNECT] backup target size mismatch: backup=%zu motors=%zu",
                  backup_targets_.size(), motors_.size());
    }

    can_ok_ = true;
    last_reconnect_attempt_ = this->now();
    last_command_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "[RECONNECT] CAN Reconnected Successfully");
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "[RECONNECT] failed: %s", e.what());
    last_reconnect_attempt_ = this->now();
  }
}

void HardwareNode::currentControl(MotorUnit &m)
{
  const double target_current = std::clamp(m.target, m.current_min, m.current_max);

  if (!sendCurrentCommand(m.id, target_current))
    throw std::runtime_error("sendCurrentCommand failed");
}

void HardwareNode::speedControl(MotorUnit &m)
{
  const double target_speed_radps = std::clamp(m.target, m.speed_min, m.speed_max);

  if (std::abs(target_speed_radps) < m.speed_deadzone)
  {
    if (!m.stopped)
    {
      if (!sendSpeedCommandRadps(m, 0.0))
        throw std::runtime_error("sendSpeedCommandRadps(0) failed");

      m.stopped = true;
    }
    return;
  }

  if (!sendSpeedCommandRadps(m, target_speed_radps))
    throw std::runtime_error("sendSpeedCommandRadps failed");

  m.stopped = false;
}

void HardwareNode::positionControl(MotorUnit &m)
{
  const double target_position_rad =
      std::clamp(m.target, m.position_min, m.position_max);

  const double raw_target_position_deg =
      softwarePositionRadToRawPositionDeg(m, target_position_rad);

  if (!sendPositionCommandDegrees(m.id, raw_target_position_deg))
    throw std::runtime_error("sendPositionCommandDegrees failed");

  m.stopped = false;
}

void HardwareNode::commandCallback(const MotorCommandArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->commands.size() != motors_.size())
  {
    RCLCPP_WARN(this->get_logger(),
                "[CMD_CB] Command size mismatch: expected %zu, got %zu",
                motors_.size(), msg->commands.size());
    return;
  }

  for (const auto &cmd : msg->commands)
  {
    auto it = std::find_if(
        motors_.begin(),
        motors_.end(),
        [&](const MotorUnit &m)
        { return m.id == cmd.id; });

    if (it == motors_.end())
    {
      RCLCPP_WARN(this->get_logger(),
                  "[CMD_CB] Unknown motor id in command: %d",
                  cmd.id);
      return;
    }
  }

  last_command_time_ = this->now();

  for (const auto &cmd : msg->commands)
  {
    auto it = std::find_if(
        motors_.begin(),
        motors_.end(),
        [&](const MotorUnit &m)
        { return m.id == cmd.id; });

    it->target = cmd.target;
  }
}

bool HardwareNode::sendCurrentCommand(int id, double current_a)
{
  const std::int32_t value =
      static_cast<std::int32_t>(std::llround(current_a * 1000.0));

  can_frame frame{};
  frame.can_id = ((CAN_PACKET_SET_CURRENT << 8) | id) | CAN_EFF_FLAG;
  frame.can_dlc = 4;

  frame.data[0] = (value >> 24) & 0xFF;
  frame.data[1] = (value >> 16) & 0xFF;
  frame.data[2] = (value >> 8) & 0xFF;
  frame.data[3] = value & 0xFF;

  if (!driver_->writeFrame(frame))
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[TX][CURRENT] write failed id=%d errno=%d (%s)",
                 id, errno, std::strerror(errno));
    return false;
  }

  return true;
}

bool HardwareNode::sendSpeedCommandRadps(const MotorUnit &m, double target_speed_radps)
{
  const double target_speed_rpm = target_speed_radps * RADPS2RPM;
  const double target_erpm =
      target_speed_rpm * m.gear_ratio * static_cast<double>(m.pole_pairs);

  const std::int32_t erpm =
      static_cast<std::int32_t>(std::llround(target_erpm));

  can_frame frame{};
  frame.can_id = ((CAN_PACKET_SET_RPM << 8) | m.id) | CAN_EFF_FLAG;
  frame.can_dlc = 4;

  frame.data[0] = (erpm >> 24) & 0xFF;
  frame.data[1] = (erpm >> 16) & 0xFF;
  frame.data[2] = (erpm >> 8) & 0xFF;
  frame.data[3] = erpm & 0xFF;

  if (!driver_->writeFrame(frame))
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[TX][SPEED] write failed id=%d errno=%d (%s)",
                 m.id, errno, std::strerror(errno));
    return false;
  }

  return true;
}

bool HardwareNode::sendPositionCommandDegrees(int id, double target_position_deg)
{
  const std::int32_t value =
      static_cast<std::int32_t>(std::llround(target_position_deg * 1000000.0));

  can_frame frame{};
  frame.can_id = ((CAN_PACKET_SET_POS << 8) | id) | CAN_EFF_FLAG;
  frame.can_dlc = 4;

  frame.data[0] = (value >> 24) & 0xFF;
  frame.data[1] = (value >> 16) & 0xFF;
  frame.data[2] = (value >> 8) & 0xFF;
  frame.data[3] = value & 0xFF;

  if (!driver_->writeFrame(frame))
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[TX][POSITION] write failed id=%d errno=%d (%s)",
                 id, errno, std::strerror(errno));
    return false;
  }

  return true;
}

bool HardwareNode::sendBrakeCommand(int id, double brake_current_a)
{
  const std::int32_t value =
      static_cast<std::int32_t>(std::llround(brake_current_a * 1000.0));

  can_frame frame{};
  frame.can_id = ((CAN_PACKET_SET_CURRENT_BRAKE << 8) | id) | CAN_EFF_FLAG;
  frame.can_dlc = 4;

  frame.data[0] = (value >> 24) & 0xFF;
  frame.data[1] = (value >> 16) & 0xFF;
  frame.data[2] = (value >> 8) & 0xFF;
  frame.data[3] = value & 0xFF;

  if (!driver_->writeFrame(frame))
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[TX][BRAKE] write failed id=%d errno=%d (%s)",
                 id, errno, std::strerror(errno));
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}