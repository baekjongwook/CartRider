#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int16.hpp"

class RawSerialPort
{
public:
    RawSerialPort(const std::string &port, int baudrate);
    ~RawSerialPort();

    RawSerialPort(const RawSerialPort &) = delete;
    RawSerialPort &operator=(const RawSerialPort &) = delete;
    RawSerialPort(RawSerialPort &&other) noexcept;
    RawSerialPort &operator=(RawSerialPort &&other) noexcept;

    bool writeBytes(const std::vector<uint8_t> &data) const;
    bool readBytes(std::vector<uint8_t> &data, int timeout_ms = 100) const;
    int fd() const noexcept;

private:
    bool configure(int baudrate);

private:
    int fd_{-1};
};

enum class ControlMode
{
    POSITION
};

struct MightyZapStatusCache
{
    bool position_valid{false};
    bool voltage_valid{false};

    uint16_t position_raw{0};
    double position_normalized{0.0};

    double voltage_v{0.0};

    rclcpp::Time last_update;
};

struct ActuatorUnit
{
    int id{0};
    ControlMode mode{ControlMode::POSITION};

    uint16_t position_min{0};
    uint16_t position_max{4095};

    uint16_t speed_min{0};
    uint16_t speed_max{1023};
    uint16_t speed_limit{1023};

    uint16_t initial_position{0};
    uint16_t target_position{0};
    uint16_t last_sent_position{0};

    uint16_t position_tolerance{20};

    bool force_enabled{false};
    bool stopped{true};

    bool command_received{false};
    bool position_command_pending{false};

    bool motion_active{false};
    bool target_reached{false};

    rclcpp::Time motion_done_time;

    MightyZapStatusCache state;
};

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode();
    ~HardwareNode();

private:
    void createActuatorsFromParameters();

    void controlLoop();
    void stateLoop();

    void goalPositionCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void goalSpeedCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void forceEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void shutdownAllActuators();
    void attemptReconnect();

    void positionControl(ActuatorUnit &actuator);
    void checkMotionCompletionTimeout();

    bool echo(int id);
    bool forceEnable(int id, bool enable);
    bool sendGoalPosition(int id, uint16_t position);
    bool sendGoalSpeed(int id, uint16_t speed);

    bool readPresentPosition(int id, uint16_t &position);
    bool readPresentVoltage(int id, double &voltage);

    bool store1Byte(int id, uint8_t address, uint8_t value);
    bool store2Byte(int id, uint8_t address, uint16_t value);
    bool loadData(int id, uint8_t address, uint8_t length, std::vector<uint8_t> &data);

    std::vector<uint8_t> makePacket(
        uint8_t id,
        uint8_t command,
        const std::vector<uint8_t> &factors = {}) const;

    uint8_t calcChecksum(
        uint8_t id,
        uint8_t size,
        uint8_t command,
        const std::vector<uint8_t> &factors) const;

    bool writePacket(const std::vector<uint8_t> &packet);
    bool readPacket(std::vector<uint8_t> &response, int timeout_ms = 100);

    ActuatorUnit *findActuatorById(int id);

    static uint16_t clampPosition(uint16_t value, uint16_t min_value, uint16_t max_value);
    static uint16_t clampSpeed(uint16_t value, uint16_t min_value, uint16_t max_value);

private:
    static constexpr uint8_t CMD_ECHO = 0xF1;
    static constexpr uint8_t CMD_LOAD_DATA = 0xF2;
    static constexpr uint8_t CMD_STORE_DATA = 0xF3;

    static constexpr uint8_t ADDR_FORCE_ON_OFF = 0x80;
    static constexpr uint8_t ADDR_GOAL_POSITION = 0x86;
    static constexpr uint8_t ADDR_PRESENT_POSITION = 0x8C;
    static constexpr uint8_t ADDR_PRESENT_VOLTAGE = 0x92;

    static constexpr uint8_t ADDR_GOAL_SPEED = 0x15;

private:
    std::string serial_port_;
    int baudrate_{57600};
    int command_timeout_ms_{2000};

    std::unique_ptr<RawSerialPort> driver_;

    std::vector<ActuatorUnit> actuators_;
    std::vector<uint16_t> backup_targets_;

    bool serial_ok_{true};

    rclcpp::Time last_command_time_;
    rclcpp::Time last_reconnect_attempt_;

    std::mutex mutex_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr goal_position_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr goal_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr force_enable_sub_;

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr present_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr present_voltage_pub_;
};