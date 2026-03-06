// RMD ROM Config Node
// 2026.02.18 백종욱

#include <rclcpp/rclcpp.hpp>
#include <myactuator_rmd/myactuator_rmd.hpp>
#include <array>
#include <thread>

#define MOTOR_ID 25

#define CLEAR_MULTI_TRUN_VALUE 0x01
#define CANID_FILTER_ENABLE 0x02
#define ERROR_STATUS_TRANSMISSON_ENABLE 0x03
#define THE_MULTI_TURN_VALUE_IS_SAVED_WHEN_THE_POWER_IS_OFF 0x04
#define SET_CANID 0x05
#define SET_THE_MAXIMUM_POSITIVE_ANGLE_FOR_THE_POSITION_OPERATION_MODE 0x06
#define SET_THE_MAXIMUM_NEGATIVE_ANGLE_FOR_THE_POSITION_OPERATION_MODE 0x07

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    myactuator_rmd::can::Node node("can0");
    uint32_t can_id = 0x140 + MOTOR_ID;

    uint8_t index = SET_CANID;
    uint32_t value = 2;

    std::array<uint8_t,8> data{};
    data[0] = 0x20;
    data[1] = index;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = value & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = (value >> 16) & 0xFF;
    data[7] = (value >> 24) & 0xFF;

    node.write(can_id, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "RMD Configuration completed.\n" << std::endl;

    rclcpp::shutdown();

    return 0;
}
