// Frontbot Test Node
// 2026.04.17
//
// 역할:
// - yaml 안 읽음
// - hardware_node는 별도 launch로 이미 떠 있다고 가정
// - 터미널 입력으로 rmd_command / vesc_command 직접 publish
//
// 입력 방식:
// - RMD motor id 1 : RPM 입력
// - VESC motor id 3 : DEG 입력
//
// 고정 출력:
// - RMD motor id 2 : 항상 0
// - VESC motor id 4 : 항상 0
//
// publish 단위:
// - RMD  : rad/s
// - VESC : rad

#include <rclcpp/rclcpp.hpp>

#include "cartrider_rmd_sdk/msg/motor_command.hpp"
#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"

#include "cartrider_vesc_sdk/msg/motor_command.hpp"
#include "cartrider_vesc_sdk/msg/motor_command_array.hpp"

#include <atomic>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
    constexpr double RPM_TO_RADPS = 2.0 * M_PI / 60.0;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
}

class FrontbotTestNode : public rclcpp::Node
{
public:
    FrontbotTestNode()
        : Node("frontbot_test_node")
    {
        this->declare_parameter<int>("rmd_active_id", 1);
        this->declare_parameter<int>("rmd_zero_id", 2);
        this->declare_parameter<int>("vesc_active_id", 3);
        this->declare_parameter<int>("vesc_zero_id", 4);
        this->declare_parameter<int>("publish_rate_hz", 10);

        rmd_active_id_ = this->get_parameter("rmd_active_id").as_int();
        rmd_zero_id_ = this->get_parameter("rmd_zero_id").as_int();
        vesc_active_id_ = this->get_parameter("vesc_active_id").as_int();
        vesc_zero_id_ = this->get_parameter("vesc_zero_id").as_int();
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_int();

        rmd_pub_ = this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>("rmd_command", 10);
        vesc_pub_ = this->create_publisher<cartrider_vesc_sdk::msg::MotorCommandArray>("vesc_command", 10);

        printHelp();

        pub_thread_ = std::thread(&FrontbotTestNode::publishLoop, this);
        input_thread_ = std::thread(&FrontbotTestNode::inputLoop, this);

        RCLCPP_INFO(this->get_logger(), "frontbot_test_node started.");
        RCLCPP_INFO(this->get_logger(), "Active RMD id: %d, Zero RMD id: %d", rmd_active_id_, rmd_zero_id_);
        RCLCPP_INFO(this->get_logger(), "Active VESC id: %d, Zero VESC id: %d", vesc_active_id_, vesc_zero_id_);
    }

    ~FrontbotTestNode() override
    {
        running_ = false;

        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
        if (pub_thread_.joinable())
        {
            pub_thread_.join();
        }
    }

private:
    void printHelp()
    {
        std::cout << "\n================ frontbot_test_node ================\n";
        std::cout << "Direct test publisher for one swerve module\n\n";
        std::cout << "Published topics:\n";
        std::cout << "  /rmd_command\n";
        std::cout << "  /vesc_command\n\n";
        std::cout << "User input units:\n";
        std::cout << "  RMD  active motor : RPM\n";
        std::cout << "  VESC active motor : DEG\n\n";
        std::cout << "Published units:\n";
        std::cout << "  RMD  : rad/s\n";
        std::cout << "  VESC : rad\n\n";
        std::cout << "Fixed behavior:\n";
        std::cout << "  RMD  zero motor target is always 0\n";
        std::cout << "  VESC zero motor target is always 0\n\n";
        std::cout << "Menu:\n";
        std::cout << "  1 : set both RMD(id1) and VESC(id3)\n";
        std::cout << "  2 : set RMD only\n";
        std::cout << "  3 : set VESC only\n";
        std::cout << "  4 : zero active targets\n";
        std::cout << "  5 : print current targets\n";
        std::cout << "  q : quit\n";
        std::cout << "====================================================\n";
    }

    void inputLoop()
    {
        while (rclcpp::ok() && running_)
        {
            std::string cmd;
            std::cout << "\nSelect command (1/2/3/4/5/q): ";
            std::cin >> cmd;

            if (!std::cin)
            {
                resetInput();
                continue;
            }

            if (cmd == "1")
            {
                setBothTargets();
            }
            else if (cmd == "2")
            {
                setRmdTarget();
            }
            else if (cmd == "3")
            {
                setVescTarget();
            }
            else if (cmd == "4")
            {
                zeroActiveTargets();
            }
            else if (cmd == "5")
            {
                printCurrentTargets();
            }
            else if (cmd == "q" || cmd == "Q")
            {
                RCLCPP_INFO(this->get_logger(), "Input loop terminated by user.");
                running_ = false;
                rclcpp::shutdown();
                return;
            }
            else
            {
                std::cout << "Unknown command.\n";
            }
        }
    }

    void setBothTargets()
    {
        double new_rmd_rpm = 0.0;
        double new_vesc_deg = 0.0;

        std::cout << "\n[RMD] Enter target for motor " << rmd_active_id_ << " [RPM]: ";
        if (!readDouble(new_rmd_rpm))
            return;

        std::cout << "[VESC] Enter target for motor " << vesc_active_id_ << " [DEG]: ";
        if (!readDouble(new_vesc_deg))
            return;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            rmd_target_rpm_ = new_rmd_rpm;
            vesc_target_deg_ = new_vesc_deg;
        }

        printCurrentTargets();
    }

    void setRmdTarget()
    {
        double new_rmd_rpm = 0.0;

        std::cout << "\n[RMD] Enter target for motor " << rmd_active_id_ << " [RPM]: ";
        if (!readDouble(new_rmd_rpm))
            return;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            rmd_target_rpm_ = new_rmd_rpm;
        }

        printCurrentTargets();
    }

    void setVescTarget()
    {
        double new_vesc_deg = 0.0;

        std::cout << "\n[VESC] Enter target for motor " << vesc_active_id_ << " [DEG]: ";
        if (!readDouble(new_vesc_deg))
            return;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            vesc_target_deg_ = new_vesc_deg;
        }

        printCurrentTargets();
    }

    void zeroActiveTargets()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            rmd_target_rpm_ = 0.0;
            vesc_target_deg_ = 0.0;
        }

        RCLCPP_INFO(this->get_logger(), "Active targets set to zero.");
        printCurrentTargets();
    }

    void printCurrentTargets()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "\n------------- Current Targets -------------\n";
        std::cout << "RMD  motor " << rmd_active_id_ << " : "
                  << rmd_target_rpm_ << " [RPM]  -> "
                  << (rmd_target_rpm_ * RPM_TO_RADPS) << " [rad/s]\n";

        std::cout << "RMD  motor " << rmd_zero_id_
                  << " : 0.0000 [RPM]  -> 0.0000 [rad/s]\n";

        std::cout << "VESC motor " << vesc_active_id_ << " : "
                  << vesc_target_deg_ << " [DEG]  -> "
                  << (vesc_target_deg_ * DEG_TO_RAD) << " [rad]\n";

        std::cout << "VESC motor " << vesc_zero_id_
                  << " : 0.0000 [DEG]  -> 0.0000 [rad]\n";
        std::cout << "-------------------------------------------\n";
    }

    bool readDouble(double &value)
    {
        std::cin >> value;
        if (!std::cin)
        {
            std::cout << "Invalid input.\n";
            resetInput();
            return false;
        }
        return true;
    }

    void resetInput()
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    void publishLoop()
    {
        rclcpp::Rate rate(std::max(1, publish_rate_hz_));

        while (rclcpp::ok() && running_)
        {
            double rmd_target_rpm = 0.0;
            double vesc_target_deg = 0.0;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                rmd_target_rpm = rmd_target_rpm_;
                vesc_target_deg = vesc_target_deg_;
            }

            const double rmd_target_radps = rmd_target_rpm * RPM_TO_RADPS;
            const double vesc_target_rad = vesc_target_deg * DEG_TO_RAD;

            cartrider_rmd_sdk::msg::MotorCommandArray rmd_msg;
            cartrider_vesc_sdk::msg::MotorCommandArray vesc_msg;

            // RMD active motor
            {
                cartrider_rmd_sdk::msg::MotorCommand cmd;
                cmd.id = rmd_active_id_;
                cmd.target = rmd_target_radps;
                rmd_msg.commands.push_back(cmd);
            }

            // RMD zero motor
            {
                cartrider_rmd_sdk::msg::MotorCommand cmd;
                cmd.id = rmd_zero_id_;
                cmd.target = 0.0;
                rmd_msg.commands.push_back(cmd);
            }

            // VESC active motor
            {
                cartrider_vesc_sdk::msg::MotorCommand cmd;
                cmd.id = vesc_active_id_;
                cmd.target = vesc_target_rad;
                vesc_msg.commands.push_back(cmd);
            }

            // VESC zero motor
            {
                cartrider_vesc_sdk::msg::MotorCommand cmd;
                cmd.id = vesc_zero_id_;
                cmd.target = 0.0;
                vesc_msg.commands.push_back(cmd);
            }

            rmd_pub_->publish(rmd_msg);
            vesc_pub_->publish(vesc_msg);

            rate.sleep();
        }
    }

private:
    rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr rmd_pub_;
    rclcpp::Publisher<cartrider_vesc_sdk::msg::MotorCommandArray>::SharedPtr vesc_pub_;

    int rmd_active_id_{1};
    int rmd_zero_id_{2};
    int vesc_active_id_{3};
    int vesc_zero_id_{4};
    int publish_rate_hz_{10};

    double rmd_target_rpm_{0.0};
    double vesc_target_deg_{0.0};

    std::mutex mutex_;
    std::thread pub_thread_;
    std::thread input_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FrontbotTestNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}