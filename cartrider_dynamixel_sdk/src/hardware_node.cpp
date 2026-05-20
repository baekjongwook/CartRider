#include "cartrider_dynamixel_sdk/hardware_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

DynamixelHardwareNode::DynamixelHardwareNode()
    : Node("hardware_node")
{
    serial_port_ = this->declare_parameter<std::string>(
        "dynamixel_serial_port");

    baudrate_ = this->declare_parameter<int>(
        "dynamixel_baudrate");

    motor_ids_ = this->declare_parameter<std::vector<int64_t>>(
        "dynamixel_motor_ids");

    position_min_ = this->declare_parameter<std::vector<double>>(
        "dynamixel_position_min");

    position_max_ = this->declare_parameter<std::vector<double>>(
        "dynamixel_position_max");

    position_omega_max_ = this->declare_parameter<std::vector<double>>(
        "dynamixel_position_omega_max");

    initial_position_ = this->declare_parameter<std::vector<double>>(
        "dynamixel_initial_position");

    torque_enable_at_start_ = this->declare_parameter<bool>(
        "dynamixel_torque_enable_at_start");

    state_publish_rate_hz_ = this->declare_parameter<double>(
        "dynamixel_state_publish_rate_hz");

    validateParameters();

    port_handler_ = dynamixel::PortHandler::getPortHandler(serial_port_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!port_handler_->openPort())
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "Failed to open Dynamixel port: %s",
            serial_port_.c_str());

        throw std::runtime_error("Failed to open Dynamixel port");
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Succeeded to open Dynamixel port: %s",
        serial_port_.c_str());

    if (!port_handler_->setBaudRate(baudrate_))
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "Failed to set Dynamixel baudrate: %d",
            baudrate_);

        port_handler_->closePort();

        throw std::runtime_error("Failed to set Dynamixel baudrate");
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Succeeded to set Dynamixel baudrate: %d",
        baudrate_);

    if (torque_enable_at_start_)
    {
        for (const auto id_raw : motor_ids_)
        {
            writeTorqueEnable(static_cast<uint8_t>(id_raw), true);
        }
    }

    writeInitialPosition();

    command_sub_ =
        this->create_subscription<cartrider_dynamixel_sdk::msg::MotorCommandArray>(
            "dynamixel_command",
            10,
            std::bind(
                &DynamixelHardwareNode::commandCallback,
                this,
                std::placeholders::_1));

    state_pub_ =
        this->create_publisher<cartrider_dynamixel_sdk::msg::MotorStateArray>(
            "dynamixel_state",
            10);

    const auto period_ms =
        std::max(1, static_cast<int>(1000.0 / state_publish_rate_hz_));

    state_timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&DynamixelHardwareNode::publishState, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Dynamixel Hardware Node Started. port=%s baudrate=%d protocol=%.1f motors=%zu",
        serial_port_.c_str(),
        baudrate_,
        PROTOCOL_VERSION,
        motor_ids_.size());
}

DynamixelHardwareNode::~DynamixelHardwareNode()
{
    if (packet_handler_ != nullptr && port_handler_ != nullptr)
    {
        for (const auto id_raw : motor_ids_)
        {
            writeTorqueEnable(static_cast<uint8_t>(id_raw), false);
        }
    }

    if (port_handler_ != nullptr)
    {
        port_handler_->closePort();
    }
}

void DynamixelHardwareNode::validateParameters()
{
    const std::size_t n = motor_ids_.size();

    if (n == 0)
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "dynamixel_motor_ids is empty.");

        throw std::runtime_error("dynamixel_motor_ids is empty");
    }

    if (position_min_.size() != n ||
        position_max_.size() != n ||
        position_omega_max_.size() != n ||
        initial_position_.size() != n)
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "Dynamixel parameter size mismatch. ids=%zu min=%zu max=%zu omega_max=%zu initial=%zu",
            motor_ids_.size(),
            position_min_.size(),
            position_max_.size(),
            position_omega_max_.size(),
            initial_position_.size());

        throw std::runtime_error("Dynamixel parameter size mismatch");
    }

    for (std::size_t i = 0; i < n; ++i)
    {
        if (motor_ids_[i] < 0 || motor_ids_[i] > 253)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Invalid Dynamixel ID: %ld",
                motor_ids_[i]);

            throw std::runtime_error("Invalid Dynamixel ID");
        }

        position_min_[i] =
            std::clamp(position_min_[i], 0.0, 1023.0);

        position_max_[i] =
            std::clamp(position_max_[i], 0.0, 1023.0);

        position_omega_max_[i] =
            std::clamp(position_omega_max_[i], 0.0, 1023.0);

        if (position_min_[i] > position_max_[i])
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Invalid position range for ID %ld: min=%.3f max=%.3f",
                motor_ids_[i],
                position_min_[i],
                position_max_[i]);

            throw std::runtime_error("Invalid Dynamixel position range");
        }

        initial_position_[i] =
            std::clamp(
                initial_position_[i],
                position_min_[i],
                position_max_[i]);
    }

    if (serial_port_.empty())
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "dynamixel_serial_port is empty.");

        throw std::runtime_error("dynamixel_serial_port is empty");
    }

    if (baudrate_ <= 0)
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "Invalid baudrate: %d",
            baudrate_);

        throw std::runtime_error("Invalid baudrate");
    }

    if (state_publish_rate_hz_ <= 0.0)
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "Invalid dynamixel_state_publish_rate_hz: %.3f",
            state_publish_rate_hz_);

        throw std::runtime_error("Invalid dynamixel_state_publish_rate_hz");
    }
}

int DynamixelHardwareNode::findMotorIndex(int32_t id) const
{
    for (std::size_t i = 0; i < motor_ids_.size(); ++i)
    {
        if (static_cast<int32_t>(motor_ids_[i]) == id)
        {
            return static_cast<int>(i);
        }
    }

    return -1;
}

bool DynamixelHardwareNode::checkTxRxResult(
    int dxl_comm_result,
    uint8_t dxl_error,
    uint8_t id,
    const std::string &operation)
{
    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[ID:%u][%s] TxRx failed: %s",
            id,
            operation.c_str(),
            packet_handler_->getTxRxResult(dxl_comm_result));

        return false;
    }

    if (dxl_error != 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[ID:%u][%s] Packet error: %s",
            id,
            operation.c_str(),
            packet_handler_->getRxPacketError(dxl_error));

        return false;
    }

    return true;
}

bool DynamixelHardwareNode::writeTorqueEnable(
    uint8_t id,
    bool enable)
{
    uint8_t dxl_error = 0;

    const int dxl_comm_result =
        packet_handler_->write1ByteTxRx(
            port_handler_,
            id,
            ADDR_TORQUE_ENABLE,
            enable ? TORQUE_ENABLE : TORQUE_DISABLE,
            &dxl_error);

    const bool ok =
        checkTxRxResult(
            dxl_comm_result,
            dxl_error,
            id,
            enable ? "torque_enable" : "torque_disable");

    if (ok)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[ID:%u] Torque %s",
            id,
            enable ? "enabled" : "disabled");
    }

    return ok;
}

bool DynamixelHardwareNode::write2Byte(
    uint8_t id,
    uint16_t address,
    uint16_t value,
    const std::string &operation)
{
    uint8_t dxl_error = 0;

    const int dxl_comm_result =
        packet_handler_->write2ByteTxRx(
            port_handler_,
            id,
            address,
            value,
            &dxl_error);

    return checkTxRxResult(
        dxl_comm_result,
        dxl_error,
        id,
        operation);
}

bool DynamixelHardwareNode::read2Byte(
    uint8_t id,
    uint16_t address,
    uint16_t &value,
    const std::string &operation)
{
    uint8_t dxl_error = 0;

    const int dxl_comm_result =
        packet_handler_->read2ByteTxRx(
            port_handler_,
            id,
            address,
            &value,
            &dxl_error);

    return checkTxRxResult(
        dxl_comm_result,
        dxl_error,
        id,
        operation);
}

void DynamixelHardwareNode::writeInitialPosition()
{
    for (std::size_t i = 0; i < motor_ids_.size(); ++i)
    {
        const uint8_t id =
            static_cast<uint8_t>(motor_ids_[i]);

        const uint16_t moving_speed =
            static_cast<uint16_t>(
                std::lround(
                    std::clamp(
                        position_omega_max_[i],
                        0.0,
                        1023.0)));

        const uint16_t goal_position =
            static_cast<uint16_t>(
                std::lround(
                    std::clamp(
                        initial_position_[i],
                        position_min_[i],
                        position_max_[i])));

        const bool speed_ok =
            write2Byte(
                id,
                ADDR_MOVING_SPEED,
                moving_speed,
                "write_initial_moving_speed");

        const bool position_ok =
            write2Byte(
                id,
                ADDR_GOAL_POSITION,
                goal_position,
                "write_initial_goal_position");

        if (speed_ok && position_ok)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[INIT][ID:%u] goal_position=%u moving_speed=%u",
                id,
                goal_position,
                moving_speed);
        }
    }
}

void DynamixelHardwareNode::commandCallback(
    const cartrider_dynamixel_sdk::msg::MotorCommandArray::SharedPtr msg)
{
    for (const auto &command : msg->commands)
    {
        const int index = findMotorIndex(command.id);

        if (index < 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Ignored command for unknown Dynamixel ID: %d",
                command.id);

            continue;
        }

        const uint8_t id =
            static_cast<uint8_t>(command.id);

        const uint16_t goal_position =
            static_cast<uint16_t>(
                std::lround(
                    std::clamp(
                        command.target,
                        position_min_[index],
                        position_max_[index])));

        const uint16_t moving_speed =
            static_cast<uint16_t>(
                std::lround(
                    std::clamp(
                        position_omega_max_[index],
                        0.0,
                        1023.0)));

        const bool speed_ok =
            write2Byte(
                id,
                ADDR_MOVING_SPEED,
                moving_speed,
                "write_moving_speed");

        const bool position_ok =
            write2Byte(
                id,
                ADDR_GOAL_POSITION,
                goal_position,
                "write_goal_position");

        if (speed_ok && position_ok)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[CMD][ID:%u] target=%.3f goal_position=%u moving_speed=%u",
                id,
                command.target,
                goal_position,
                moving_speed);
        }
    }
}

void DynamixelHardwareNode::publishState()
{
    cartrider_dynamixel_sdk::msg::MotorStateArray state_array;

    for (const auto id_raw : motor_ids_)
    {
        const uint8_t id = static_cast<uint8_t>(id_raw);

        cartrider_dynamixel_sdk::msg::MotorState state;
        state.id = static_cast<int32_t>(id);

        uint16_t present_position = 0;
        uint16_t present_speed = 0;
        uint16_t present_load = 0;

        bool ok = true;

        ok = read2Byte(
                 id,
                 ADDR_PRESENT_POSITION,
                 present_position,
                 "read_present_position") &&
             ok;

        ok = read2Byte(
                 id,
                 ADDR_PRESENT_SPEED,
                 present_speed,
                 "read_present_speed") &&
             ok;

        ok = read2Byte(
                 id,
                 ADDR_PRESENT_LOAD,
                 present_load,
                 "read_present_load") &&
             ok;

        if (!ok)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[STATE][ID:%u] Failed to read Dynamixel state.",
                id);
        }

        state.position = static_cast<double>(present_position);
        state.speed = static_cast<double>(present_speed);
        state.current = static_cast<double>(present_load);

        state_array.states.push_back(state);
    }

    state_pub_->publish(state_array);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelHardwareNode>());
    rclcpp::shutdown();
    return 0;
}