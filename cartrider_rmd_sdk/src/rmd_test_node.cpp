// RMD Test Node
// 2026.02.18 백종욱

#include <rclcpp/rclcpp.hpp>
#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <sstream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <limits> 

class RmdTestNode : public rclcpp::Node
{
public:
  RmdTestNode() : Node("rmd_test_node")
  {
    loadYamlConfig();

    command_pub_ = this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>("rmd_command", 10);

    pub_thread_ = std::thread(&RmdTestNode::publishLoop, this);
  }

  ~RmdTestNode()
  {
    running_ = false;
    if (pub_thread_.joinable())
      pub_thread_.join();
  }

  void run()
  {
    while (rclcpp::ok() && running_)
    {
      cartrider_rmd_sdk::msg::MotorCommandArray msg;

      for (size_t i = 0; i < motor_ids_.size(); ++i)
      {
        int id = motor_ids_[i];
        const std::string & mode = operate_modes_[i];

        double target = 0.0;

        std::cout << "\nMotor " << id << " (";

        if (mode == "current")
        {
          std::cout << "current mode)\n";
          std::cout << "  Enter target current [A] (" << current_min_[i] << " ~ " << current_max_[i] << "): ";

          std::cin >> target;

          if (!std::cin)
          {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(get_logger(), "Invalid input. Try again.");
            --i;
            continue;
          }

          target = std::clamp(target, current_min_[i], current_max_[i]);
        }
        else if (mode == "speed")
        {
          std::cout << "speed mode)\n";
          std::cout << "  Enter target speed [RPM] (" << speed_min_[i] << " ~ " << speed_max_[i] << "): ";

          std::cin >> target;

          if (!std::cin)
          {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(get_logger(), "Invalid input. Try again.");
            --i;
            continue;
          }

          target = std::clamp(target, speed_min_[i], speed_max_[i]);
          target = target * 2.0 * M_PI / 60.0;
        }
        else if (mode == "position")
        {
          std::cout << "position mode)\n";
          std::cout << "  Enter target position [DEG] (" << position_min_[i] << " ~ " << position_max_[i] << "): ";

          std::cin >> target;

          if (!std::cin)
          {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(get_logger(), "Invalid input. Try again.");
            --i;
            continue;
          }

          target = std::clamp(target, position_min_[i], position_max_[i]);
          target = target * M_PI / 180.0;
        }

        cartrider_rmd_sdk::msg::MotorCommand cmd;
        cmd.id = id;
        cmd.target = target;

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

  void loadYamlConfig()
  {
    try
    {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("cartrider_rmd_sdk");

      std::string yaml_path = pkg_share + "/param/motors.yaml";

      YAML::Node config = YAML::LoadFile(yaml_path);

      YAML::Node params = config["hardware_node"]["ros__parameters"];

      motor_ids_ = params["motor_ids"].as<std::vector<int64_t>>();
      operate_modes_ = params["operate_modes"].as<std::vector<std::string>>();

      current_min_ = params["current_min"].as<std::vector<double>>();
      current_max_ = params["current_max"].as<std::vector<double>>();

      speed_min_ = params["speed_min"].as<std::vector<double>>();
      speed_max_ = params["speed_max"].as<std::vector<double>>();

      position_min_ = params["position_min"].as<std::vector<double>>();
      position_max_ = params["position_max"].as<std::vector<double>>();

      validateParameters();
    }
    catch (const std::exception & e)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to load motors.yaml: %s", e.what());
      throw;
    }
  }

  void validateParameters()
  {
    size_t n = motor_ids_.size();

    if (n == 0)
      throw std::runtime_error("No motor_ids defined in YAML.");

    if (operate_modes_.size() != n ||
        current_min_.size() != n ||
        current_max_.size() != n ||
        speed_min_.size() != n ||
        speed_max_.size() != n ||
        position_min_.size() != n ||
        position_max_.size() != n)
    {
      throw std::runtime_error("Parameter size mismatch in YAML.");
    }
  }

  void publishLoop()
  {
    rclcpp::Rate rate(10); 

    while (rclcpp::ok() && running_)
    {
      cartrider_rmd_sdk::msg::MotorCommandArray msg_copy;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_copy = current_msg_;
      }

      if (!msg_copy.commands.empty())
        command_pub_->publish(msg_copy);

      rate.sleep();
    }
  }

  rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr command_pub_;

  std::vector<int64_t> motor_ids_;
  std::vector<std::string> operate_modes_;

  std::vector<double> current_min_;
  std::vector<double> current_max_;

  std::vector<double> speed_min_;
  std::vector<double> speed_max_;

  std::vector<double> position_min_;
  std::vector<double> position_max_;

  cartrider_rmd_sdk::msg::MotorCommandArray current_msg_;
  std::mutex mutex_;

  std::thread pub_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RmdTestNode>();
  std::thread input_thread([&]() {node->run();});
  rclcpp::spin(node);
  rclcpp::shutdown();

  if (input_thread.joinable())
    input_thread.join();

  return 0;
}