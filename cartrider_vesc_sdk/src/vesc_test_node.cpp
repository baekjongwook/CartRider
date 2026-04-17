// VESC Test Node
// 2026.03.24 백종욱

#include <rclcpp/rclcpp.hpp>
#include "cartrider_vesc_sdk/msg/motor_command_array.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <limits>
#include <string>
#include <cmath>

class VescTestNode : public rclcpp::Node
{
public:
  VescTestNode() : Node("vesc_test_node")
  {
    loadYamlConfig();

    command_pub_ =
        this->create_publisher<cartrider_vesc_sdk::msg::MotorCommandArray>("vesc_command", 10);

    pub_thread_ = std::thread(&VescTestNode::publishLoop, this);
  }

  ~VescTestNode()
  {
    running_ = false;
    if (pub_thread_.joinable())
      pub_thread_.join();
  }

  void run()
  {
    while (rclcpp::ok() && running_)
    {
      cartrider_vesc_sdk::msg::MotorCommandArray msg;

      for (size_t i = 0; i < motor_ids_.size(); ++i)
      {
        const int id = static_cast<int>(motor_ids_[i]);
        const std::string &mode = operate_modes_[i];

        double user_input_value = 0.0;
        double target_si = 0.0;

        std::cout << "\nMotor " << id << " (";

        if (mode == "current")
        {
          std::cout << "current mode)\n";
          std::cout << "  Enter target current [A] ("
                    << current_min_[i] << " ~ " << current_max_[i] << "): ";

          std::cin >> user_input_value;

          if (!std::cin)
          {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(get_logger(), "Invalid input. Try again.");
            --i;
            continue;
          }

          user_input_value = std::clamp(user_input_value, current_min_[i], current_max_[i]);
          target_si = user_input_value;
        }
        else if (mode == "speed")
        {
          std::cout << "speed mode)\n";
          std::cout << "  Enter target speed [RPM] ("
                    << speed_min_rpm_[i] << " ~ " << speed_max_rpm_[i] << "): ";

          std::cin >> user_input_value;

          if (!std::cin)
          {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(get_logger(), "Invalid input. Try again.");
            --i;
            continue;
          }

          user_input_value = std::clamp(user_input_value, speed_min_rpm_[i], speed_max_rpm_[i]);

          if (std::abs(user_input_value) < speed_deadzone_rpm_[i])
            user_input_value = 0.0;

          target_si = rpmToRadps(user_input_value);
        }
        else if (mode == "position")
        {
          std::cout << "position mode)\n";
          std::cout << "  Enter target position [DEG] ("
                    << position_min_deg_[i] << " ~ " << position_max_deg_[i] << "): ";

          std::cin >> user_input_value;

          if (!std::cin)
          {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(get_logger(), "Invalid input. Try again.");
            --i;
            continue;
          }

          user_input_value =
              std::clamp(user_input_value, position_min_deg_[i], position_max_deg_[i]);

          target_si = degToRad(user_input_value);
        }
        else
        {
          RCLCPP_WARN(get_logger(), "Unknown mode '%s' for motor %d", mode.c_str(), id);
          continue;
        }

        cartrider_vesc_sdk::msg::MotorCommand cmd;
        cmd.id = id;
        cmd.target = target_si;

        msg.commands.push_back(cmd);
      }

      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      {
        std::lock_guard<std::mutex> lock(mutex_);
        current_msg_ = msg;
      }

      RCLCPP_INFO(get_logger(), "Command updated.");
    }
  }

private:
  static constexpr double PI = 3.14159265358979323846;
  static constexpr double DEG2RAD = PI / 180.0;
  static constexpr double RPM2RADPS = 2.0 * PI / 60.0;

  static double degToRad(double deg)
  {
    return deg * DEG2RAD;
  }

  static double rpmToRadps(double rpm)
  {
    return rpm * RPM2RADPS;
  }

  void loadYamlConfig()
  {
    try
    {
      std::string pkg_share =
          ament_index_cpp::get_package_share_directory("cartrider_vesc_sdk");
      std::string yaml_path = pkg_share + "/param/motors.yaml";

      YAML::Node config = YAML::LoadFile(yaml_path);
      YAML::Node params = config["hardware_node"]["ros__parameters"];

      motor_ids_ = params["vesc_motor_ids"].as<std::vector<int64_t>>();
      operate_modes_ = params["vesc_operate_modes"].as<std::vector<std::string>>();

      current_min_ = params["vesc_current_min"].as<std::vector<double>>();
      current_max_ = params["vesc_current_max"].as<std::vector<double>>();

      speed_min_rpm_ = params["vesc_speed_min"].as<std::vector<double>>();
      speed_max_rpm_ = params["vesc_speed_max"].as<std::vector<double>>();
      speed_deadzone_rpm_ = params["vesc_speed_deadzone"].as<std::vector<double>>();

      position_min_deg_ = params["vesc_position_min"].as<std::vector<double>>();
      position_max_deg_ = params["vesc_position_max"].as<std::vector<double>>();

      validateParameters();
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to load motors.yaml: %s", e.what());
      throw;
    }
  }

  void validateParameters()
  {
    const size_t n = motor_ids_.size();

    if (n == 0)
      throw std::runtime_error("No vesc_motor_ids defined in YAML.");

    if (operate_modes_.size() != n ||
        current_min_.size() != n ||
        current_max_.size() != n ||
        speed_min_rpm_.size() != n ||
        speed_max_rpm_.size() != n ||
        speed_deadzone_rpm_.size() != n ||
        position_min_deg_.size() != n ||
        position_max_deg_.size() != n)
    {
      throw std::runtime_error("Parameter size mismatch in YAML.");
    }
  }

  void publishLoop()
  {
    rclcpp::Rate rate(10);

    while (rclcpp::ok() && running_)
    {
      cartrider_vesc_sdk::msg::MotorCommandArray msg_copy;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_copy = current_msg_;
      }

      if (!msg_copy.commands.empty())
        command_pub_->publish(msg_copy);

      rate.sleep();
    }
  }

  rclcpp::Publisher<cartrider_vesc_sdk::msg::MotorCommandArray>::SharedPtr command_pub_;

  std::vector<int64_t> motor_ids_;
  std::vector<std::string> operate_modes_;

  std::vector<double> current_min_;
  std::vector<double> current_max_;

  std::vector<double> speed_min_rpm_;
  std::vector<double> speed_max_rpm_;
  std::vector<double> speed_deadzone_rpm_;

  std::vector<double> position_min_deg_;
  std::vector<double> position_max_deg_;

  cartrider_vesc_sdk::msg::MotorCommandArray current_msg_;
  std::mutex mutex_;

  std::thread pub_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VescTestNode>();
  std::thread input_thread([&]()
                           { node->run(); });
  rclcpp::spin(node);
  rclcpp::shutdown();

  if (input_thread.joinable())
    input_thread.join();

  return 0;
}