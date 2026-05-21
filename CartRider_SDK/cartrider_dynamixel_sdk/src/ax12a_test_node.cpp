// AX-12A Test Node
// Pure position command test

#include <rclcpp/rclcpp.hpp>

#include "cartrider_dynamixel_sdk/msg/motor_command_array.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <limits>
#include <stdexcept>

class Ax12aTestNode : public rclcpp::Node
{
public:
    Ax12aTestNode()
        : Node("ax12a_test_node")
    {
        loadYamlConfig();

        command_pub_ =
            this->create_publisher<cartrider_dynamixel_sdk::msg::MotorCommandArray>(
                "dynamixel_command",
                10);

        pub_thread_ = std::thread(&Ax12aTestNode::publishLoop, this);
    }

    ~Ax12aTestNode()
    {
        running_ = false;

        if (pub_thread_.joinable())
        {
            pub_thread_.join();
        }
    }

    void run()
    {
        while (rclcpp::ok() && running_)
        {
            cartrider_dynamixel_sdk::msg::MotorCommandArray msg;

            for (std::size_t i = 0; i < motor_ids_.size(); ++i)
            {
                const int id = motor_ids_[i];

                double target = 0.0;

                std::cout << "\nMotor " << id << " (position mode)\n";
                std::cout << "  Enter target position [raw] ("
                          << position_min_[i] << " ~ "
                          << position_max_[i] << "): ";

                std::cin >> target;

                if (!std::cin)
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                    RCLCPP_WARN(
                        this->get_logger(),
                        "Invalid input. Try again.");

                    --i;
                    continue;
                }

                target =
                    std::clamp(
                        target,
                        position_min_[i],
                        position_max_[i]);

                cartrider_dynamixel_sdk::msg::MotorCommand cmd;
                cmd.id = id;
                cmd.target = target;

                msg.commands.push_back(cmd);
            }

            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            {
                std::lock_guard<std::mutex> lock(mutex_);
                current_msg_ = msg;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Dynamixel position command updated.");
        }
    }

private:
    void loadYamlConfig()
    {
        try
        {
            const std::string pkg_share =
                ament_index_cpp::get_package_share_directory("cartrider_dynamixel_sdk");

            const std::string yaml_path =
                pkg_share + "/param/motors.yaml";

            YAML::Node config = YAML::LoadFile(yaml_path);

            YAML::Node params;

            if (config["hardware_node"] && config["hardware_node"]["ros__parameters"])
            {
                params = config["hardware_node"]["ros__parameters"];
            }
            else if (config["/**"] && config["/**"]["ros__parameters"])
            {
                params = config["/**"]["ros__parameters"];
            }
            else
            {
                throw std::runtime_error(
                    "Missing hardware_node.ros__parameters or /**.ros__parameters in motors.yaml.");
            }

            motor_ids_ =
                params["dynamixel_motor_ids"].as<std::vector<int64_t>>();

            position_min_ =
                params["dynamixel_position_min"].as<std::vector<double>>();

            position_max_ =
                params["dynamixel_position_max"].as<std::vector<double>>();

            position_omega_max_ =
                params["dynamixel_position_omega_max"].as<std::vector<double>>();

            validateParameters();

            RCLCPP_INFO(
                this->get_logger(),
                "Loaded AX-12A motors.yaml. motors=%zu",
                motor_ids_.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Failed to load motors.yaml: %s",
                e.what());

            throw;
        }
    }

    void validateParameters()
    {
        const std::size_t n = motor_ids_.size();

        if (n == 0)
        {
            throw std::runtime_error("No dynamixel_motor_ids defined in YAML.");
        }

        if (position_min_.size() != n ||
            position_max_.size() != n ||
            position_omega_max_.size() != n)
        {
            throw std::runtime_error("Parameter size mismatch in YAML.");
        }

        for (std::size_t i = 0; i < n; ++i)
        {
            if (motor_ids_[i] < 0 || motor_ids_[i] > 253)
            {
                throw std::runtime_error("Invalid dynamixel_motor_ids in YAML.");
            }

            position_min_[i] =
                std::clamp(position_min_[i], 0.0, 1023.0);

            position_max_[i] =
                std::clamp(position_max_[i], 0.0, 1023.0);

            position_omega_max_[i] =
                std::clamp(position_omega_max_[i], 0.0, 1023.0);

            if (position_min_[i] > position_max_[i])
            {
                std::swap(position_min_[i], position_max_[i]);
            }
        }
    }

    void publishLoop()
    {
        rclcpp::Rate rate(10);

        while (rclcpp::ok() && running_)
        {
            cartrider_dynamixel_sdk::msg::MotorCommandArray msg_copy;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                msg_copy = current_msg_;
            }

            if (!msg_copy.commands.empty())
            {
                command_pub_->publish(msg_copy);
            }

            rate.sleep();
        }
    }

private:
    rclcpp::Publisher<cartrider_dynamixel_sdk::msg::MotorCommandArray>::SharedPtr command_pub_;

    std::vector<int64_t> motor_ids_;

    std::vector<double> position_min_;
    std::vector<double> position_max_;
    std::vector<double> position_omega_max_;

    cartrider_dynamixel_sdk::msg::MotorCommandArray current_msg_;

    std::mutex mutex_;
    std::thread pub_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Ax12aTestNode>();

    std::thread input_thread([&]()
                             { node->run(); });

    rclcpp::spin(node);
    rclcpp::shutdown();

    if (input_thread.joinable())
    {
        input_thread.join();
    }

    return 0;
}