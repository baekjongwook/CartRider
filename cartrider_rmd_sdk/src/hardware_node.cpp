// Hardware Node
// 2026.02.18 백종욱

#include "cartrider_rmd_sdk/hardware_node.hpp"

HardwareNode::HardwareNode() : rclcpp::Node("hardware_node")
{
  RCLCPP_INFO(this->get_logger(), "Hardware Node Started");

  this->declare_parameter("rmd_can_interface", "can0");
  this->declare_parameter("command_timeout_ms", 2000);

  this->declare_parameter("rmd_motor_ids", std::vector<int64_t>{});
  this->declare_parameter("rmd_operate_modes", std::vector<std::string>{});

  this->declare_parameter("rmd_gear_ratio", std::vector<double>{});

  this->declare_parameter("rmd_current_min", std::vector<double>{});
  this->declare_parameter("rmd_current_max", std::vector<double>{});

  this->declare_parameter("rmd_speed_min", std::vector<double>{});
  this->declare_parameter("rmd_speed_max", std::vector<double>{});
  this->declare_parameter("rmd_speed_deadzone", std::vector<double>{});

  this->declare_parameter("rmd_position_min", std::vector<double>{});
  this->declare_parameter("rmd_position_max", std::vector<double>{});
  this->declare_parameter("rmd_position_omega_max", std::vector<double>{});

  this->declare_parameter("rmd_safety_current_min", std::vector<double>{});
  this->declare_parameter("rmd_safety_current_max", std::vector<double>{});

  this->declare_parameter("rmd_safety_speed_min", std::vector<double>{});
  this->declare_parameter("rmd_safety_speed_max", std::vector<double>{});

  this->declare_parameter("rmd_safety_position_min", std::vector<double>{});
  this->declare_parameter("rmd_safety_position_max", std::vector<double>{});
  this->declare_parameter("rmd_safety_position_omega_max", std::vector<double>{});

  can_interface_ = this->get_parameter("rmd_can_interface").as_string();
  command_timeout_ms_ = this->get_parameter("command_timeout_ms").as_int();

  driver_ = std::make_unique<myactuator_rmd::CanDriver>(can_interface_);

  createMotorsFromParameters();

  last_command_time_ = this->now();
  last_reconnect_attempt_ = this->now();

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HardwareNode::controlLoop, this));

  state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&HardwareNode::stateLoop, this));

  subscription_ = this->create_subscription<cartrider_rmd_sdk::msg::MotorCommandArray>(
      "rmd_command",
      10,
      std::bind(&HardwareNode::commandCallback, this, std::placeholders::_1));

  publication_ = this->create_publisher<cartrider_rmd_sdk::msg::MotorStateArray>(
      "rmd_state",
      10);
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
  for (auto &m : motors_)
  {
    if (m.motor)
    {
      try
      {
        m.motor->shutdownMotor();
        m.stopped = true;
      }
      catch (...)
      {
      }
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void HardwareNode::createMotorsFromParameters()
{
  motors_.clear();

  std::vector<int64_t> motor_ids;
  std::vector<std::string> operate_modes;

  std::vector<double> gear_ratio;

  std::vector<double> current_min, current_max;
  std::vector<double> speed_min, speed_max, speed_deadzone;
  std::vector<double> position_min, position_max, position_omega_max;

  std::vector<double> safety_current_min, safety_current_max;
  std::vector<double> safety_speed_min, safety_speed_max;
  std::vector<double> safety_position_min, safety_position_max, safety_position_omega_max;

  try
  {
    this->get_parameter("rmd_motor_ids", motor_ids);
    this->get_parameter("rmd_operate_modes", operate_modes);

    this->get_parameter("rmd_gear_ratio", gear_ratio);

    this->get_parameter("rmd_current_min", current_min);
    this->get_parameter("rmd_current_max", current_max);

    this->get_parameter("rmd_speed_min", speed_min);
    this->get_parameter("rmd_speed_max", speed_max);
    this->get_parameter("rmd_speed_deadzone", speed_deadzone);

    this->get_parameter("rmd_position_min", position_min);
    this->get_parameter("rmd_position_max", position_max);
    this->get_parameter("rmd_position_omega_max", position_omega_max);

    this->get_parameter("rmd_safety_current_min", safety_current_min);
    this->get_parameter("rmd_safety_current_max", safety_current_max);

    this->get_parameter("rmd_safety_speed_min", safety_speed_min);
    this->get_parameter("rmd_safety_speed_max", safety_speed_max);

    this->get_parameter("rmd_safety_position_min", safety_position_min);
    this->get_parameter("rmd_safety_position_max", safety_position_max);
    this->get_parameter("rmd_safety_position_omega_max", safety_position_omega_max);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter loading failed: %s", e.what());
    return;
  }

  size_t n = motor_ids.size();

  if (n == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "No motors defined!");
    return;
  }

  if (operate_modes.size() != n)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter size mismatch: rmd_operate_modes");
    return;
  }

  auto check_size = [&](const std::vector<double> &v, const std::string &name)
  {
    if (v.size() != n)
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter size mismatch: %s", name.c_str());
      return false;
    }
    return true;
  };

  if (!check_size(gear_ratio, "rmd_gear_ratio") ||
      !check_size(current_min, "rmd_current_min") ||
      !check_size(current_max, "rmd_current_max") ||
      !check_size(speed_min, "rmd_speed_min") ||
      !check_size(speed_max, "rmd_speed_max") ||
      !check_size(speed_deadzone, "rmd_speed_deadzone") ||
      !check_size(position_min, "rmd_position_min") ||
      !check_size(position_max, "rmd_position_max") ||
      !check_size(position_omega_max, "rmd_position_omega_max") ||
      !check_size(safety_current_min, "rmd_safety_current_min") ||
      !check_size(safety_current_max, "rmd_safety_current_max") ||
      !check_size(safety_speed_min, "rmd_safety_speed_min") ||
      !check_size(safety_speed_max, "rmd_safety_speed_max") ||
      !check_size(safety_position_min, "rmd_safety_position_min") ||
      !check_size(safety_position_max, "rmd_safety_position_max") ||
      !check_size(safety_position_omega_max, "rmd_safety_position_omega_max"))
  {
    return;
  }

  for (size_t i = 0; i < n; ++i)
  {
    MotorUnit unit;

    unit.id = motor_ids[i];

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

    if (gear_ratio[i] <= 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid gear_ratio for motor %ld: %.3f",
                   motor_ids[i], gear_ratio[i]);
      return;
    }

    const double applied_current_min = std::max(current_min[i], safety_current_min[i]);
    const double applied_current_max = std::min(current_max[i], safety_current_max[i]);

    const double applied_speed_min = std::max(speed_min[i], safety_speed_min[i]);
    const double applied_speed_max = std::min(speed_max[i], safety_speed_max[i]);

    const double applied_position_min = std::max(position_min[i], safety_position_min[i]);
    const double applied_position_max = std::min(position_max[i], safety_position_max[i]);

    const double applied_position_omega_max =
        std::min(position_omega_max[i], safety_position_omega_max[i]);

    if (applied_current_min > applied_current_max ||
        applied_speed_min > applied_speed_max ||
        applied_position_min > applied_position_max)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid applied limit range for motor %ld", motor_ids[i]);
      return;
    }

    unit.gear_ratio = gear_ratio[i];

    unit.current_min = applied_current_min;
    unit.current_max = applied_current_max;

    unit.speed_min = applied_speed_min * 2.0 * M_PI / 60.0;
    unit.speed_max = applied_speed_max * 2.0 * M_PI / 60.0;
    unit.speed_deadzone = speed_deadzone[i] * 2.0 * M_PI / 60.0;

    unit.position_min = applied_position_min * M_PI / 180.0;
    unit.position_max = applied_position_max * M_PI / 180.0;
    unit.position_omega_max = applied_position_omega_max * 2.0 * M_PI / 60.0;

    unit.motor = std::make_unique<myactuator_rmd::ActuatorInterface>(*driver_, unit.id);

    unit.target = 0.0;
    unit.stopped = true;

    motors_.push_back(std::move(unit));
  }

  RCLCPP_INFO(this->get_logger(), "Initialized %zu motors", motors_.size());
}

void HardwareNode::controlLoop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!can_ok_)
  {
    attemptReconnect();
    return;
  }

  if ((this->now() - last_command_time_).nanoseconds() / 1e6 > command_timeout_ms_)
  {
    shutdownAllMotors();
    return;
  }

  try
  {
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
  catch (const myactuator_rmd::can::SocketException &e)
  {
    int err = e.code().value();

    RCLCPP_ERROR(this->get_logger(), "CAN error: %s", e.what());

    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)
      return;

    backup_targets_.clear();

    for (auto &m : motors_)
    {
      backup_targets_.push_back({m.mode, m.target});
      try
      {
        m.motor->shutdownMotor();
      }
      catch (...)
      {
      }
    }

    can_ok_ = false;
    last_reconnect_attempt_ = this->now();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "CAN fatal error: %s", e.what());

    backup_targets_.clear();

    for (auto &m : motors_)
    {
      backup_targets_.push_back({m.mode, m.target});
      try
      {
        m.motor->shutdownMotor();
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
  {
    return;
  }

  cartrider_rmd_sdk::msg::MotorStateArray state_msg;

  try
  {
    for (auto &m : motors_)
    {
      auto status = m.motor->getMotorStatus2();

      cartrider_rmd_sdk::msg::MotorState s;
      s.id = m.id;
      s.current = status.current;
      s.speed = (status.shaft_speed * M_PI / 180.0) / m.gear_ratio;
      s.position = (m.motor->getMultiTurnAngle() * M_PI / 180.0) / m.gear_ratio;

      state_msg.states.push_back(s);
    }

    publication_->publish(state_msg);
  }
  catch (const myactuator_rmd::can::SocketException &e)
  {
    int err = e.code().value();

    RCLCPP_ERROR(this->get_logger(), "CAN error: %s", e.what());

    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)
      return;

    can_ok_ = false;
    last_reconnect_attempt_ = this->now();
  }
  catch (const std::exception &e)
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
    driver_ = std::make_unique<myactuator_rmd::CanDriver>(can_interface_);

    createMotorsFromParameters();

    if (backup_targets_.size() == motors_.size())
    {
      for (size_t i = 0; i < motors_.size(); ++i)
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
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Reconnect failed: %s", e.what());
    last_reconnect_attempt_ = this->now();
  }
}

void HardwareNode::currentControl(MotorUnit &m)
{
  if (!m.motor)
    return;

  double target_current = std::clamp(m.target, m.current_min, m.current_max);

  m.motor->sendCurrentSetpoint(target_current);
}

void HardwareNode::speedControl(MotorUnit &m)
{
  if (!m.motor)
    return;

  double target_output_w = std::clamp(m.target, m.speed_min, m.speed_max);
  double target_motor_w = target_output_w * m.gear_ratio;
  double target_dps = target_motor_w * 180.0 / M_PI;

  if (std::abs(target_output_w) < m.speed_deadzone)
  {
    if (!m.stopped)
    {
      m.motor->shutdownMotor();
      m.stopped = true;
    }
    return;
  }

  m.motor->sendVelocitySetpoint(target_dps);
  m.stopped = false;
}

void HardwareNode::positionControl(MotorUnit &m)
{
  if (!m.motor)
    return;

  double target_output_position_rad =
      std::clamp(m.target, m.position_min, m.position_max);

  double target_motor_position_rad =
      target_output_position_rad * m.gear_ratio;

  double target_position_deg = target_motor_position_rad * 180.0 / M_PI;
  double omega_max_dps = (m.position_omega_max * m.gear_ratio) * 180.0 / M_PI;

  m.motor->sendPositionAbsoluteSetpoint(target_position_deg, omega_max_dps);
}

void HardwareNode::commandCallback(const cartrider_rmd_sdk::msg::MotorCommandArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->commands.size() != motors_.size())
  {
    RCLCPP_WARN(this->get_logger(), "Command size mismatch: expected %zu, got %zu",
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
      RCLCPP_WARN(this->get_logger(), "Unknown motor id in command: %d", cmd.id);
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