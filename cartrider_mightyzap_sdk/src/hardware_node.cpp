// Hardware Node for mightyZAP Linear Actuator
// 2026.05.18 백종욱

#include "cartrider_mightyzap_sdk/hardware_node.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <thread>

using std_msgs::msg::Bool;
using std_msgs::msg::Float32;
using std_msgs::msg::UInt16;

namespace
{
    constexpr uint16_t MIGHTYZAP_POSITION_MIN = 0;
    constexpr uint16_t MIGHTYZAP_POSITION_MAX = 4095;
    constexpr uint16_t MIGHTYZAP_SPEED_MIN = 0;
    constexpr uint16_t MIGHTYZAP_SPEED_MAX = 1023;
    constexpr uint16_t MIGHTYZAP_OPERATING_RATE_MIN = 0;
    constexpr uint16_t MIGHTYZAP_OPERATING_RATE_MAX = 1023;

    speed_t toTermiosBaudrate(int baudrate)
    {
        switch (baudrate)
        {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        default:
            throw std::runtime_error("Unsupported baudrate");
        }
    }
}

RawSerialPort::RawSerialPort(const std::string &port, int baudrate)
{
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (fd_ < 0)
    {
        throw std::runtime_error(
            std::string("Failed to open serial port: ") + port + " / " + std::strerror(errno));
    }

    if (!configure(baudrate))
    {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("Failed to configure serial port");
    }

    tcflush(fd_, TCIOFLUSH);
}

RawSerialPort::~RawSerialPort()
{
    if (fd_ >= 0)
        ::close(fd_);
}

RawSerialPort::RawSerialPort(RawSerialPort &&other) noexcept
    : fd_(other.fd_)
{
    other.fd_ = -1;
}

RawSerialPort &RawSerialPort::operator=(RawSerialPort &&other) noexcept
{
    if (this != &other)
    {
        if (fd_ >= 0)
            ::close(fd_);

        fd_ = other.fd_;
        other.fd_ = -1;
    }

    return *this;
}

bool RawSerialPort::configure(int baudrate)
{
    if (fd_ < 0)
        return false;

    termios tty{};

    if (tcgetattr(fd_, &tty) != 0)
        return false;

    const speed_t speed = toTermiosBaudrate(baudrate);

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        return false;

    return true;
}

bool RawSerialPort::writeBytes(const std::vector<uint8_t> &data) const
{
    if (fd_ < 0)
        return false;

    const ssize_t written = ::write(fd_, data.data(), data.size());

    if (written < 0)
        return false;

    if (static_cast<std::size_t>(written) != data.size())
        return false;

    tcdrain(fd_);
    return true;
}

bool RawSerialPort::readBytes(std::vector<uint8_t> &data, int timeout_ms) const
{
    data.clear();

    if (fd_ < 0)
        return false;

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    const int poll_result = poll(&pfd, 1, timeout_ms);

    if (poll_result <= 0)
        return false;

    uint8_t buffer[128]{};
    const ssize_t n = ::read(fd_, buffer, sizeof(buffer));

    if (n <= 0)
        return false;

    data.assign(buffer, buffer + n);
    return true;
}

int RawSerialPort::fd() const noexcept
{
    return fd_;
}

HardwareNode::HardwareNode()
    : rclcpp::Node("hardware_node")
{
    RCLCPP_INFO(this->get_logger(), "mightyZAP Hardware Node Started");

    this->declare_parameter("mightyzap_serial_port", "/dev/rs485_front_mightyzap");
    this->declare_parameter("mightyzap_baudrate", 57600);
    this->declare_parameter("mightyzap_command_timeout_ms", 2000);

    this->declare_parameter("mightyzap_actuator_id", 0);

    this->declare_parameter("mightyzap_position_min", 0);
    this->declare_parameter("mightyzap_position_max", 4095);
    this->declare_parameter("mightyzap_position_tolerance", 20);
    this->declare_parameter("mightyzap_initial_position", 2700);

    this->declare_parameter("mightyzap_speed_min", 0);
    this->declare_parameter("mightyzap_speed_max", 1023);
    this->declare_parameter("mightyzap_initial_speed", 600);

    this->declare_parameter("mightyzap_safe_position", 2700);
    this->declare_parameter("mightyzap_operating_rate_limit", 500);

    this->declare_parameter("mightyzap_force_on_at_start", true);

    serial_port_ = this->get_parameter("mightyzap_serial_port").as_string();
    baudrate_ = this->get_parameter("mightyzap_baudrate").as_int();
    command_timeout_ms_ = this->get_parameter("mightyzap_command_timeout_ms").as_int();

    try
    {
        driver_ = std::make_unique<RawSerialPort>(serial_port_, baudrate_);
        serial_ok_ = true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[INIT] Serial open failed: %s", e.what());
        serial_ok_ = false;
    }

    createActuatorsFromParameters();

    last_command_time_ = this->now();
    last_reconnect_attempt_ = this->now();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&HardwareNode::controlLoop, this));

    state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&HardwareNode::stateLoop, this));

    goal_position_sub_ = this->create_subscription<UInt16>(
        "goal_position",
        10,
        std::bind(&HardwareNode::goalPositionCallback, this, std::placeholders::_1));

    goal_speed_sub_ = this->create_subscription<UInt16>(
        "goal_speed",
        10,
        std::bind(&HardwareNode::goalSpeedCallback, this, std::placeholders::_1));

    force_enable_sub_ = this->create_subscription<Bool>(
        "force_enable",
        10,
        std::bind(&HardwareNode::forceEnableCallback, this, std::placeholders::_1));

    present_position_pub_ = this->create_publisher<UInt16>("present_position", 10);
    present_voltage_pub_ = this->create_publisher<Float32>("present_voltage", 10);
    present_operating_rate_pub_ = this->create_publisher<UInt16>("present_operating_rate", 10);
    fault_pub_ = this->create_publisher<Bool>("mightyzap_fault", 10);

    RCLCPP_INFO(
        this->get_logger(),
        "[INIT] serial_port=%s baudrate=%d idle_force_off_ms=%d actuators=%zu",
        serial_port_.c_str(),
        baudrate_,
        command_timeout_ms_,
        actuators_.size());

    if (serial_ok_)
    {
        for (auto &actuator : actuators_)
        {
            if (echo(actuator.id))
            {
                RCLCPP_INFO(this->get_logger(), "[INIT] Echo success id=%d", actuator.id);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[INIT] Echo failed id=%d", actuator.id);
            }

            if (sendGoalSpeed(actuator.id, actuator.speed_limit))
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[INIT] Initial speed set id=%d speed=%u",
                    actuator.id,
                    actuator.speed_limit);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[INIT] Initial speed set failed id=%d", actuator.id);
            }

            if (this->get_parameter("mightyzap_force_on_at_start").as_bool())
            {
                if (forceEnable(actuator.id, true))
                {
                    actuator.force_enabled = true;
                    RCLCPP_INFO(this->get_logger(), "[INIT] Force enabled id=%d", actuator.id);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "[INIT] Force enable failed id=%d", actuator.id);
                }
            }
        }
    }

    publishFault(false);
}

HardwareNode::~HardwareNode()
{
    if (timer_)
        timer_->cancel();

    if (state_timer_)
        state_timer_->cancel();

    std::lock_guard<std::mutex> lock(mutex_);
    shutdownAllActuators();
}

void HardwareNode::createActuatorsFromParameters()
{
    actuators_.clear();

    const int actuator_id = this->get_parameter("mightyzap_actuator_id").as_int();

    int position_min = this->get_parameter("mightyzap_position_min").as_int();
    int position_max = this->get_parameter("mightyzap_position_max").as_int();
    int position_tolerance = this->get_parameter("mightyzap_position_tolerance").as_int();
    int initial_position = this->get_parameter("mightyzap_initial_position").as_int();

    int speed_min = this->get_parameter("mightyzap_speed_min").as_int();
    int speed_max = this->get_parameter("mightyzap_speed_max").as_int();
    int initial_speed = this->get_parameter("mightyzap_initial_speed").as_int();

    int safe_position = this->get_parameter("mightyzap_safe_position").as_int();
    int operating_rate_limit = this->get_parameter("mightyzap_operating_rate_limit").as_int();

    ActuatorUnit unit;

    unit.id = std::clamp(actuator_id, 0, 253);

    position_min = std::clamp(position_min, 0, 4095);
    position_max = std::clamp(position_max, 0, 4095);

    if (position_min > position_max)
        std::swap(position_min, position_max);

    position_tolerance = std::clamp(position_tolerance, 0, 4095);
    initial_position = std::clamp(initial_position, position_min, position_max);
    safe_position = std::clamp(safe_position, position_min, position_max);

    speed_min = std::clamp(speed_min, 0, 1023);
    speed_max = std::clamp(speed_max, 0, 1023);

    if (speed_min > speed_max)
        std::swap(speed_min, speed_max);

    initial_speed = std::clamp(initial_speed, speed_min, speed_max);

    operating_rate_limit = std::clamp(
        operating_rate_limit,
        static_cast<int>(MIGHTYZAP_OPERATING_RATE_MIN),
        static_cast<int>(MIGHTYZAP_OPERATING_RATE_MAX));

    unit.position_min = static_cast<uint16_t>(position_min);
    unit.position_max = static_cast<uint16_t>(position_max);
    unit.position_tolerance = static_cast<uint16_t>(position_tolerance);
    unit.initial_position = static_cast<uint16_t>(initial_position);

    unit.speed_min = static_cast<uint16_t>(speed_min);
    unit.speed_max = static_cast<uint16_t>(speed_max);
    unit.speed_limit = static_cast<uint16_t>(initial_speed);

    unit.safe_position = static_cast<uint16_t>(safe_position);
    unit.operating_rate_limit = static_cast<uint16_t>(operating_rate_limit);

    unit.target_position = unit.initial_position;
    unit.last_sent_position = unit.position_min;

    unit.force_enabled = false;
    unit.stopped = false;

    unit.command_received = true;
    unit.position_command_pending = true;

    unit.motion_active = true;
    unit.target_reached = false;

    unit.fault_active = false;
    unit.safety_return_active = false;

    unit.motion_done_time = this->now();

    actuators_.push_back(unit);

    backup_targets_.assign(actuators_.size(), unit.target_position);

    RCLCPP_INFO(
        this->get_logger(),
        "[INIT] mightyZAP id=%d | position[%u, %u] initial=%u safe=%u tolerance=%u | speed[%u, %u] initial=%u | operating_rate_limit=%u",
        unit.id,
        unit.position_min,
        unit.position_max,
        unit.initial_position,
        unit.safe_position,
        unit.position_tolerance,
        unit.speed_min,
        unit.speed_max,
        unit.speed_limit,
        unit.operating_rate_limit);
}

void HardwareNode::controlLoop()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!serial_ok_)
    {
        attemptReconnect();
        return;
    }

    try
    {
        for (auto &actuator : actuators_)
        {
            positionControl(actuator);
        }

        checkMotionCompletionTimeout();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[CTRL] Serial error: %s", e.what());

        backup_targets_.clear();

        for (auto &actuator : actuators_)
        {
            backup_targets_.push_back(actuator.target_position);
            actuator.position_command_pending = true;
            actuator.motion_active = true;
            actuator.target_reached = false;
        }

        serial_ok_ = false;
        last_reconnect_attempt_ = this->now();

        RCLCPP_ERROR(this->get_logger(), "[CTRL] serial_ok set false, entering reconnect mode");
    }
}

void HardwareNode::stateLoop()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!serial_ok_)
        return;

    try
    {
        for (auto &actuator : actuators_)
        {
            uint16_t position = 0;
            uint16_t operating_rate = 0;
            double voltage = 0.0;

            if (readPresentPosition(actuator.id, position))
            {
                actuator.state.position_raw = position;

                const uint16_t denom =
                    std::max<uint16_t>(1, actuator.position_max - actuator.position_min);

                actuator.state.position_normalized =
                    static_cast<double>(position - actuator.position_min) /
                    static_cast<double>(denom);

                actuator.state.position_valid = true;
                actuator.state.last_update = this->now();

                const int error =
                    std::abs(static_cast<int>(position) - static_cast<int>(actuator.target_position));

                if (actuator.motion_active &&
                    error <= static_cast<int>(actuator.position_tolerance))
                {
                    actuator.motion_active = false;
                    actuator.target_reached = true;
                    actuator.motion_done_time = this->now();

                    RCLCPP_INFO(
                        this->get_logger(),
                        "[STATE] Target reached id=%d position=%u target=%u error=%d tolerance=%u safety_return=%s",
                        actuator.id,
                        position,
                        actuator.target_position,
                        error,
                        actuator.position_tolerance,
                        actuator.safety_return_active ? "true" : "false");
                }

                UInt16 msg;
                msg.data = position;
                present_position_pub_->publish(msg);
            }

            if (readPresentOperatingRate(actuator.id, operating_rate))
            {
                actuator.state.operating_rate_raw = operating_rate;
                actuator.state.operating_rate_valid = true;
                actuator.state.last_update = this->now();

                UInt16 msg;
                msg.data = operating_rate;
                present_operating_rate_pub_->publish(msg);

                if (!actuator.safety_return_active &&
                    operating_rate > actuator.operating_rate_limit)
                {
                    startSafetyReturn(actuator, operating_rate);
                }
            }

            if (readPresentVoltage(actuator.id, voltage))
            {
                actuator.state.voltage_v = voltage;
                actuator.state.voltage_valid = true;
                actuator.state.last_update = this->now();

                Float32 msg;
                msg.data = static_cast<float>(voltage);
                present_voltage_pub_->publish(msg);
            }
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[STATE] Serial error: %s", e.what());

        serial_ok_ = false;
        last_reconnect_attempt_ = this->now();

        RCLCPP_ERROR(this->get_logger(), "[STATE] serial_ok set false");
    }
}

void HardwareNode::checkMotionCompletionTimeout()
{
    if (command_timeout_ms_ <= 0)
        return;

    for (auto &actuator : actuators_)
    {
        if (!actuator.force_enabled)
            continue;

        if (actuator.motion_active)
            continue;

        if (!actuator.target_reached)
            continue;

        const double idle_ms =
            (this->now() - actuator.motion_done_time).nanoseconds() / 1e6;

        if (idle_ms <= static_cast<double>(command_timeout_ms_))
            continue;

        RCLCPP_WARN(
            this->get_logger(),
            "[IDLE_OFF] Target reached and idle %.1f ms > %d ms. Force off id=%d safety_return=%s",
            idle_ms,
            command_timeout_ms_,
            actuator.id,
            actuator.safety_return_active ? "true" : "false");

        if (forceEnable(actuator.id, false))
        {
            actuator.force_enabled = false;
            actuator.stopped = true;
            actuator.command_received = false;
            actuator.position_command_pending = false;
            actuator.motion_active = false;
            actuator.target_reached = false;
            actuator.safety_return_active = false;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[IDLE_OFF] Force off failed id=%d", actuator.id);
        }
    }
}

void HardwareNode::startSafetyReturn(ActuatorUnit &actuator, uint16_t operating_rate)
{
    actuator.fault_active = true;
    actuator.safety_return_active = true;

    actuator.target_position = actuator.safe_position;
    actuator.position_command_pending = true;
    actuator.command_received = true;
    actuator.stopped = false;

    actuator.motion_active = true;
    actuator.target_reached = false;
    actuator.motion_done_time = this->now();

    if (!actuator.force_enabled)
    {
        if (forceEnable(actuator.id, true))
        {
            actuator.force_enabled = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[SAFETY] Force enable failed id=%d", actuator.id);
        }
    }

    if (!sendGoalSpeed(actuator.id, actuator.speed_limit))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[SAFETY] Failed to set speed id=%d speed=%u",
            actuator.id,
            actuator.speed_limit);
    }

    publishFault(true);

    RCLCPP_ERROR(
        this->get_logger(),
        "[SAFETY] Operating rate exceeded id=%d rate=%u limit=%u. Returning to safe_position=%u",
        actuator.id,
        operating_rate,
        actuator.operating_rate_limit,
        actuator.safe_position);
}

void HardwareNode::publishFault(bool fault)
{
    Bool msg;
    msg.data = fault;
    fault_pub_->publish(msg);
}

void HardwareNode::attemptReconnect()
{
    if ((this->now() - last_reconnect_attempt_).seconds() < 1.0)
        return;

    RCLCPP_WARN(this->get_logger(), "[RECONNECT] Attempting serial reconnect...");

    try
    {
        driver_.reset();
        driver_ = std::make_unique<RawSerialPort>(serial_port_, baudrate_);

        const bool had_backup = backup_targets_.size() == actuators_.size();

        createActuatorsFromParameters();

        if (had_backup && backup_targets_.size() == actuators_.size())
        {
            for (std::size_t i = 0; i < actuators_.size(); ++i)
            {
                actuators_[i].target_position = backup_targets_[i];
                actuators_[i].command_received = true;
                actuators_[i].position_command_pending = true;
                actuators_[i].motion_active = true;
                actuators_[i].target_reached = false;
                actuators_[i].stopped = false;
            }
        }

        serial_ok_ = true;
        last_reconnect_attempt_ = this->now();
        last_command_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "[RECONNECT] Serial Reconnected Successfully");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[RECONNECT] failed: %s", e.what());
        last_reconnect_attempt_ = this->now();
    }
}

void HardwareNode::shutdownAllActuators()
{
    for (auto &actuator : actuators_)
    {
        try
        {
            forceEnable(actuator.id, false);
            actuator.force_enabled = false;
            actuator.stopped = true;
            actuator.command_received = false;
            actuator.position_command_pending = false;
            actuator.motion_active = false;
            actuator.target_reached = false;
            actuator.safety_return_active = false;
        }
        catch (...)
        {
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void HardwareNode::positionControl(ActuatorUnit &actuator)
{
    if (!actuator.command_received)
        return;

    if (!actuator.position_command_pending &&
        actuator.target_position == actuator.last_sent_position)
    {
        return;
    }

    const uint16_t target_position =
        clampPosition(actuator.target_position, actuator.position_min, actuator.position_max);

    if (!sendGoalPosition(actuator.id, target_position))
        throw std::runtime_error("sendGoalPosition failed");

    actuator.last_sent_position = target_position;
    actuator.position_command_pending = false;
    actuator.stopped = false;

    actuator.motion_active = true;
    actuator.target_reached = false;
    actuator.motion_done_time = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "[CTRL] Goal position sent id=%d target=%u safety_return=%s",
        actuator.id,
        target_position,
        actuator.safety_return_active ? "true" : "false");
}

void HardwareNode::goalPositionCallback(const UInt16::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (actuators_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "[CMD_CB] No actuator available");
        return;
    }

    auto &actuator = actuators_.front();

    const uint16_t new_target =
        clampPosition(msg->data, actuator.position_min, actuator.position_max);

    actuator.target_position = new_target;
    actuator.command_received = true;
    actuator.position_command_pending = true;
    actuator.stopped = false;

    actuator.motion_active = true;
    actuator.target_reached = false;
    actuator.motion_done_time = this->now();

    actuator.safety_return_active = false;
    actuator.fault_active = false;
    publishFault(false);

    if (!actuator.force_enabled)
    {
        if (forceEnable(actuator.id, true))
        {
            actuator.force_enabled = true;
            RCLCPP_INFO(this->get_logger(), "[CMD_CB] Force auto-enabled id=%d", actuator.id);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[CMD_CB] Force auto-enable failed id=%d", actuator.id);
        }
    }

    last_command_time_ = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "[CMD_CB] Goal position id=%d target=%u",
        actuator.id,
        actuator.target_position);
}

void HardwareNode::goalSpeedCallback(const UInt16::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (actuators_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "[CMD_CB] No actuator available");
        return;
    }

    auto &actuator = actuators_.front();

    const uint16_t new_speed =
        clampSpeed(msg->data, actuator.speed_min, actuator.speed_max);

    if (new_speed == actuator.speed_limit)
    {
        last_command_time_ = this->now();
        return;
    }

    actuator.speed_limit = new_speed;

    if (!sendGoalSpeed(actuator.id, actuator.speed_limit))
    {
        RCLCPP_WARN(this->get_logger(), "[CMD_CB] Failed to set goal speed id=%d", actuator.id);
        return;
    }

    last_command_time_ = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "[CMD_CB] Goal speed id=%d speed=%u",
        actuator.id,
        actuator.speed_limit);
}

void HardwareNode::forceEnableCallback(const Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (actuators_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "[CMD_CB] No actuator available");
        return;
    }

    auto &actuator = actuators_.front();

    if (msg->data == actuator.force_enabled)
    {
        last_command_time_ = this->now();
        return;
    }

    if (!forceEnable(actuator.id, msg->data))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[CMD_CB] Failed to set force enable id=%d enable=%s",
            actuator.id,
            msg->data ? "true" : "false");
        return;
    }

    actuator.force_enabled = msg->data;

    if (!msg->data)
    {
        actuator.stopped = true;
        actuator.motion_active = false;
        actuator.target_reached = false;
        actuator.command_received = false;
        actuator.position_command_pending = false;
        actuator.safety_return_active = false;
    }

    last_command_time_ = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "[CMD_CB] Force enable id=%d enable=%s",
        actuator.id,
        msg->data ? "true" : "false");
}

ActuatorUnit *HardwareNode::findActuatorById(int id)
{
    for (auto &actuator : actuators_)
    {
        if (actuator.id == id)
            return &actuator;
    }

    return nullptr;
}

uint16_t HardwareNode::clampPosition(
    uint16_t value,
    uint16_t min_value,
    uint16_t max_value)
{
    return std::clamp<uint16_t>(value, min_value, max_value);
}

uint16_t HardwareNode::clampSpeed(
    uint16_t value,
    uint16_t min_value,
    uint16_t max_value)
{
    return std::clamp<uint16_t>(value, min_value, max_value);
}

uint8_t HardwareNode::calcChecksum(
    uint8_t id,
    uint8_t size,
    uint8_t command,
    const std::vector<uint8_t> &factors) const
{
    uint16_t sum = id + size + command;

    for (const auto b : factors)
        sum += b;

    return static_cast<uint8_t>(0xFF - (sum & 0xFF));
}

std::vector<uint8_t> HardwareNode::makePacket(
    uint8_t id,
    uint8_t command,
    const std::vector<uint8_t> &factors) const
{
    const uint8_t size = static_cast<uint8_t>(factors.size() + 2);
    const uint8_t checksum = calcChecksum(id, size, command, factors);

    std::vector<uint8_t> packet;
    packet.reserve(7 + factors.size());

    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(id);
    packet.push_back(size);
    packet.push_back(command);

    for (const auto b : factors)
        packet.push_back(b);

    packet.push_back(checksum);

    return packet;
}

bool HardwareNode::writePacket(const std::vector<uint8_t> &packet)
{
    if (!driver_)
        return false;

    if (driver_->fd() < 0)
        return false;

    tcflush(driver_->fd(), TCIFLUSH);

    if (!driver_->writeBytes(packet))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[TX] write failed errno=%d (%s)",
            errno,
            std::strerror(errno));
        return false;
    }

    return true;
}

bool HardwareNode::readPacket(std::vector<uint8_t> &response, int timeout_ms)
{
    response.clear();

    if (!driver_)
        return false;

    if (driver_->fd() < 0)
        return false;

    pollfd pfd{};
    pfd.fd = driver_->fd();
    pfd.events = POLLIN;

    const int poll_result = poll(&pfd, 1, timeout_ms);

    if (poll_result <= 0)
        return false;

    uint8_t byte = 0;
    int header_count = 0;

    const auto start = std::chrono::steady_clock::now();

    while (header_count < 3)
    {
        const ssize_t n = ::read(driver_->fd(), &byte, 1);

        if (n == 1)
        {
            if (byte == 0xFF)
            {
                response.push_back(byte);
                header_count++;
            }
            else
            {
                response.clear();
                header_count = 0;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();

        if (elapsed_ms > timeout_ms)
            return false;
    }

    uint8_t id = 0;
    uint8_t size = 0;

    if (::read(driver_->fd(), &id, 1) != 1)
        return false;

    if (::read(driver_->fd(), &size, 1) != 1)
        return false;

    response.push_back(id);
    response.push_back(size);

    const std::size_t remain = static_cast<std::size_t>(size);

    while (response.size() < 5 + remain)
    {
        const ssize_t n = ::read(driver_->fd(), &byte, 1);

        if (n == 1)
        {
            response.push_back(byte);
        }
        else
        {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();

            if (elapsed_ms > timeout_ms)
                return false;
        }
    }

    if (response.size() < 7)
        return false;

    const uint8_t rx_id = response[3];
    const uint8_t rx_size = response[4];
    const uint8_t rx_command = response[5];

    std::vector<uint8_t> factors;

    if (rx_size > 2)
    {
        const std::size_t factor_start = 6;
        const std::size_t factor_end = response.size() - 1;

        if (factor_end > factor_start)
            factors.assign(response.begin() + factor_start, response.begin() + factor_end);
    }

    const uint8_t expected_checksum =
        calcChecksum(rx_id, rx_size, rx_command, factors);

    const uint8_t received_checksum = response.back();

    if (expected_checksum != received_checksum)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[RX] checksum mismatch expected=0x%02X received=0x%02X",
            expected_checksum,
            received_checksum);
        return false;
    }

    return true;
}

bool HardwareNode::echo(int id)
{
    const auto packet = makePacket(static_cast<uint8_t>(id), CMD_ECHO);

    if (!writePacket(packet))
        return false;

    std::vector<uint8_t> response;
    return readPacket(response, 300);
}

bool HardwareNode::store1Byte(int id, uint8_t address, uint8_t value)
{
    const std::vector<uint8_t> factors = {
        address,
        value};

    const auto packet =
        makePacket(static_cast<uint8_t>(id), CMD_STORE_DATA, factors);

    return writePacket(packet);
}

bool HardwareNode::store2Byte(int id, uint8_t address, uint16_t value)
{
    const uint8_t low = static_cast<uint8_t>(value & 0xFF);
    const uint8_t high = static_cast<uint8_t>((value >> 8) & 0xFF);

    const std::vector<uint8_t> factors = {
        address,
        low,
        high};

    const auto packet =
        makePacket(static_cast<uint8_t>(id), CMD_STORE_DATA, factors);

    return writePacket(packet);
}

bool HardwareNode::loadData(
    int id,
    uint8_t address,
    uint8_t length,
    std::vector<uint8_t> &data)
{
    data.clear();

    const std::vector<uint8_t> factors = {
        address,
        length};

    const auto packet =
        makePacket(static_cast<uint8_t>(id), CMD_LOAD_DATA, factors);

    if (!writePacket(packet))
        return false;

    std::vector<uint8_t> response;

    if (!readPacket(response, 300))
        return false;

    if (response.size() < 7)
        return false;

    const uint8_t size = response[4];
    const int data_len = static_cast<int>(size) - 2;

    if (data_len <= 0)
        return false;

    const std::size_t data_start = 6;
    const std::size_t data_end = data_start + static_cast<std::size_t>(data_len);

    if (data_end > response.size() - 1)
        return false;

    data.assign(response.begin() + data_start, response.begin() + data_end);
    return true;
}

bool HardwareNode::forceEnable(int id, bool enable)
{
    return store1Byte(
        id,
        ADDR_FORCE_ON_OFF,
        enable ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0));
}

bool HardwareNode::sendGoalPosition(int id, uint16_t position)
{
    position = std::clamp<uint16_t>(
        position,
        MIGHTYZAP_POSITION_MIN,
        MIGHTYZAP_POSITION_MAX);

    return store2Byte(id, ADDR_GOAL_POSITION, position);
}

bool HardwareNode::sendGoalSpeed(int id, uint16_t speed)
{
    speed = std::clamp<uint16_t>(
        speed,
        MIGHTYZAP_SPEED_MIN,
        MIGHTYZAP_SPEED_MAX);

    return store2Byte(id, ADDR_GOAL_SPEED, speed);
}

bool HardwareNode::readPresentPosition(int id, uint16_t &position)
{
    std::vector<uint8_t> data;

    if (!loadData(id, ADDR_PRESENT_POSITION, 2, data))
        return false;

    if (data.size() < 2)
        return false;

    position =
        static_cast<uint16_t>(data[0]) |
        static_cast<uint16_t>(data[1] << 8);

    return true;
}

bool HardwareNode::readPresentOperatingRate(int id, uint16_t &operating_rate)
{
    std::vector<uint8_t> data;

    if (!loadData(id, ADDR_PRESENT_OPERATING_RATE, 2, data))
        return false;

    if (data.size() < 2)
        return false;

    operating_rate =
        static_cast<uint16_t>(data[0]) |
        static_cast<uint16_t>(data[1] << 8);

    operating_rate = std::clamp<uint16_t>(
        operating_rate,
        MIGHTYZAP_OPERATING_RATE_MIN,
        MIGHTYZAP_OPERATING_RATE_MAX);

    return true;
}

bool HardwareNode::readPresentVoltage(int id, double &voltage)
{
    std::vector<uint8_t> data;

    if (!loadData(id, ADDR_PRESENT_VOLTAGE, 1, data))
        return false;

    if (data.empty())
        return false;

    voltage = static_cast<double>(data[0]) * 0.1;
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