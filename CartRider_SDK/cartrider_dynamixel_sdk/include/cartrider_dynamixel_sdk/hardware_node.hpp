#pragma once

#include <rclcpp/rclcpp.hpp>

#include "cartrider_dynamixel_sdk/msg/motor_command_array.hpp"
#include "cartrider_dynamixel_sdk/msg/motor_state_array.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <cstdint>
#include <string>
#include <vector>

class DynamixelHardwareNode : public rclcpp::Node
{
public:
    DynamixelHardwareNode();
    ~DynamixelHardwareNode() override;

private:
    static constexpr double PROTOCOL_VERSION = 1.0;

    static constexpr uint16_t ADDR_TORQUE_ENABLE = 24;
    static constexpr uint16_t ADDR_GOAL_POSITION = 30;
    static constexpr uint16_t ADDR_MOVING_SPEED = 32;
    static constexpr uint16_t ADDR_PRESENT_POSITION = 36;
    static constexpr uint16_t ADDR_PRESENT_SPEED = 38;
    static constexpr uint16_t ADDR_PRESENT_LOAD = 40;

    static constexpr uint8_t TORQUE_DISABLE = 0;
    static constexpr uint8_t TORQUE_ENABLE = 1;

    std::string serial_port_;
    int baudrate_{0};

    std::vector<int64_t> motor_ids_;
    std::vector<double> position_min_;
    std::vector<double> position_max_;
    std::vector<double> position_omega_max_;
    std::vector<double> initial_position_;

    bool torque_enable_at_start_{false};
    double state_publish_rate_hz_{0.0};

    dynamixel::PortHandler *port_handler_{nullptr};
    dynamixel::PacketHandler *packet_handler_{nullptr};

    rclcpp::Subscription<cartrider_dynamixel_sdk::msg::MotorCommandArray>::SharedPtr command_sub_;
    rclcpp::Publisher<cartrider_dynamixel_sdk::msg::MotorStateArray>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;

private:
    void validateParameters();

    int findMotorIndex(int32_t id) const;

    bool checkTxRxResult(
        int dxl_comm_result,
        uint8_t dxl_error,
        uint8_t id,
        const std::string &operation);

    bool writeTorqueEnable(
        uint8_t id,
        bool enable);

    bool write2Byte(
        uint8_t id,
        uint16_t address,
        uint16_t value,
        const std::string &operation);

    bool read2Byte(
        uint8_t id,
        uint16_t address,
        uint16_t &value,
        const std::string &operation);

    void writeInitialPosition();

    void commandCallback(
        const cartrider_dynamixel_sdk::msg::MotorCommandArray::SharedPtr msg);

    void publishState();
};