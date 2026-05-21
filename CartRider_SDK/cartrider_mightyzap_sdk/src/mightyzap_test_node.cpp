// mightyZAP Test Node
// 2026.05.18 백종욱

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <limits>
#include <string>
#include <cstdint>
#include <stdexcept>

class MightyZapTestNode : public rclcpp::Node
{
public:
    MightyZapTestNode() : Node("mightyzap_test_node")
    {
        loadYamlConfig();

        force_pub_ =
            this->create_publisher<std_msgs::msg::Bool>("force_enable", 10);

        speed_pub_ =
            this->create_publisher<std_msgs::msg::UInt16>("goal_speed", 10);

        position_pub_ =
            this->create_publisher<std_msgs::msg::UInt16>("goal_position", 10);

        pub_thread_ = std::thread(&MightyZapTestNode::publishLoop, this);
    }

    ~MightyZapTestNode()
    {
        running_ = false;

        if (pub_thread_.joinable())
            pub_thread_.join();

        publishForce(false);
    }

    void run()
    {
        while (rclcpp::ok() && running_)
        {
            std::cout << "\n========== mightyZAP Test ==========\n";
            std::cout << "Actuator ID: " << actuator_id_ << "\n";
            std::cout << "Position range: " << position_min_ << " ~ " << position_max_ << "\n";
            std::cout << "Speed range: " << speed_min_ << " ~ " << speed_max_ << "\n";
            std::cout << "Initial position: " << initial_position_ << "\n";
            std::cout << "Initial speed: " << initial_speed_ << "\n";
            std::cout << "Safe position: " << safe_position_ << "\n";
            std::cout << "Operating rate limit: " << operating_rate_limit_ << "\n";
            std::cout << "------------------------------------\n";
            std::cout << "1. Force ON\n";
            std::cout << "2. Force OFF\n";
            std::cout << "3. Set Goal Speed\n";
            std::cout << "4. Set Goal Position\n";
            std::cout << "5. Set Speed + Position\n";
            std::cout << "6. Go Initial Position\n";
            std::cout << "7. Go Safe Position\n";
            std::cout << "8. Send Current Speed Again\n";
            std::cout << "9. Send Current Position Again\n";
            std::cout << "q. Quit\n";
            std::cout << "Select: ";

            std::string input;
            std::cin >> input;

            if (!std::cin)
            {
                clearInput();
                RCLCPP_WARN(get_logger(), "Invalid input. Try again.");
                continue;
            }

            if (input == "q" || input == "Q")
            {
                running_ = false;
                publishForce(false);
                break;
            }

            if (input == "1")
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    force_enabled_ = true;
                    force_msg_dirty_ = true;
                }

                RCLCPP_INFO(get_logger(), "Force ON requested.");
            }
            else if (input == "2")
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    force_enabled_ = false;
                    force_msg_dirty_ = true;
                }

                RCLCPP_INFO(get_logger(), "Force OFF requested.");
            }
            else if (input == "3")
            {
                int speed = readIntFromConsole(
                    "Enter goal speed [raw] (" +
                    std::to_string(speed_min_) + " ~ " +
                    std::to_string(speed_max_) + "): ");

                speed = std::clamp(speed, speed_min_, speed_max_);

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    current_speed_ = static_cast<uint16_t>(speed);
                    speed_msg_dirty_ = true;
                }

                RCLCPP_INFO(get_logger(), "Goal speed updated: %d", speed);
            }
            else if (input == "4")
            {
                int position = readIntFromConsole(
                    "Enter goal position [raw] (" +
                    std::to_string(position_min_) + " ~ " +
                    std::to_string(position_max_) + "): ");

                position = std::clamp(position, position_min_, position_max_);

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    current_position_ = static_cast<uint16_t>(position);
                    position_msg_dirty_ = true;
                }

                RCLCPP_INFO(get_logger(), "Goal position updated: %d", position);
            }
            else if (input == "5")
            {
                int speed = readIntFromConsole(
                    "Enter goal speed [raw] (" +
                    std::to_string(speed_min_) + " ~ " +
                    std::to_string(speed_max_) + "): ");

                int position = readIntFromConsole(
                    "Enter goal position [raw] (" +
                    std::to_string(position_min_) + " ~ " +
                    std::to_string(position_max_) + "): ");

                speed = std::clamp(speed, speed_min_, speed_max_);
                position = std::clamp(position, position_min_, position_max_);

                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    force_enabled_ = true;
                    current_speed_ = static_cast<uint16_t>(speed);
                    current_position_ = static_cast<uint16_t>(position);

                    force_msg_dirty_ = true;
                    speed_msg_dirty_ = true;
                    position_msg_dirty_ = true;
                }

                RCLCPP_INFO(
                    get_logger(),
                    "Force ON, goal speed=%d, goal position=%d updated.",
                    speed,
                    position);
            }
            else if (input == "6")
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    force_enabled_ = true;
                    current_speed_ = static_cast<uint16_t>(
                        std::clamp(initial_speed_, speed_min_, speed_max_));
                    current_position_ = static_cast<uint16_t>(
                        std::clamp(initial_position_, position_min_, position_max_));

                    force_msg_dirty_ = true;
                    speed_msg_dirty_ = true;
                    position_msg_dirty_ = true;
                }

                RCLCPP_INFO(
                    get_logger(),
                    "Go initial position requested. speed=%d position=%d",
                    initial_speed_,
                    initial_position_);
            }
            else if (input == "7")
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    force_enabled_ = true;
                    current_speed_ = static_cast<uint16_t>(
                        std::clamp(initial_speed_, speed_min_, speed_max_));
                    current_position_ = static_cast<uint16_t>(
                        std::clamp(safe_position_, position_min_, position_max_));

                    force_msg_dirty_ = true;
                    speed_msg_dirty_ = true;
                    position_msg_dirty_ = true;
                }

                RCLCPP_INFO(
                    get_logger(),
                    "Go safe position requested. speed=%d position=%d",
                    initial_speed_,
                    safe_position_);
            }
            else if (input == "8")
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    speed_msg_dirty_ = true;
                }

                RCLCPP_INFO(get_logger(), "Current speed publish requested.");
            }
            else if (input == "9")
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    position_msg_dirty_ = true;
                }

                RCLCPP_INFO(get_logger(), "Current position publish requested.");
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Unknown menu input: %s", input.c_str());
            }

            clearInput();
        }
    }

private:
    void loadYamlConfig()
    {
        try
        {
            std::string pkg_share =
                ament_index_cpp::get_package_share_directory("cartrider_mightyzap_sdk");

            std::string yaml_path = pkg_share + "/param/motors.yaml";

            YAML::Node config = YAML::LoadFile(yaml_path);
            YAML::Node params = config["hardware_node"]["ros__parameters"];

            if (!params)
                throw std::runtime_error("Missing hardware_node.ros__parameters in motors.yaml.");

            actuator_id_ = params["mightyzap_actuator_id"].as<int>();

            position_min_ = params["mightyzap_position_min"].as<int>();
            position_max_ = params["mightyzap_position_max"].as<int>();
            initial_position_ = params["mightyzap_initial_position"].as<int>();
            safe_position_ = params["mightyzap_safe_position"].as<int>();

            speed_min_ = params["mightyzap_speed_min"].as<int>();
            speed_max_ = params["mightyzap_speed_max"].as<int>();
            initial_speed_ = params["mightyzap_initial_speed"].as<int>();

            operating_rate_limit_ = params["mightyzap_operating_rate_limit"].as<int>();

            validateParameters();

            current_speed_ = static_cast<uint16_t>(
                std::clamp(initial_speed_, speed_min_, speed_max_));

            current_position_ = static_cast<uint16_t>(
                std::clamp(initial_position_, position_min_, position_max_));

            RCLCPP_INFO(
                this->get_logger(),
                "[YAML] actuator_id=%d position[%d, %d] initial_position=%d safe_position=%d speed[%d, %d] initial_speed=%d operating_rate_limit=%d",
                actuator_id_,
                position_min_,
                position_max_,
                initial_position_,
                safe_position_,
                speed_min_,
                speed_max_,
                initial_speed_,
                operating_rate_limit_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to load motors.yaml: %s", e.what());
            throw;
        }
    }

    void validateParameters()
    {
        if (actuator_id_ < 0 || actuator_id_ > 253)
            throw std::runtime_error("Invalid mightyzap_actuator_id in YAML.");

        position_min_ = std::clamp(position_min_, 0, 4095);
        position_max_ = std::clamp(position_max_, 0, 4095);

        if (position_min_ > position_max_)
            std::swap(position_min_, position_max_);

        initial_position_ = std::clamp(initial_position_, position_min_, position_max_);
        safe_position_ = std::clamp(safe_position_, position_min_, position_max_);

        speed_min_ = std::clamp(speed_min_, 0, 1023);
        speed_max_ = std::clamp(speed_max_, 0, 1023);

        if (speed_min_ > speed_max_)
            std::swap(speed_min_, speed_max_);

        initial_speed_ = std::clamp(initial_speed_, speed_min_, speed_max_);

        operating_rate_limit_ = std::clamp(operating_rate_limit_, 0, 1023);
    }

    void publishLoop()
    {
        rclcpp::Rate rate(10);

        while (rclcpp::ok() && running_)
        {
            bool force_enabled = false;
            uint16_t speed = 0;
            uint16_t position = 0;

            bool publish_force = false;
            bool publish_speed = false;
            bool publish_position = false;

            {
                std::lock_guard<std::mutex> lock(mutex_);

                force_enabled = force_enabled_;
                speed = current_speed_;
                position = current_position_;

                publish_force = force_msg_dirty_;
                publish_speed = speed_msg_dirty_;
                publish_position = position_msg_dirty_;

                force_msg_dirty_ = false;
                speed_msg_dirty_ = false;
                position_msg_dirty_ = false;
            }

            if (publish_force)
                publishForce(force_enabled);

            if (publish_speed)
                publishSpeed(speed);

            if (publish_position)
                publishPosition(position);

            rate.sleep();
        }
    }

    void publishForce(bool enable)
    {
        std_msgs::msg::Bool msg;
        msg.data = enable;
        force_pub_->publish(msg);
    }

    void publishSpeed(uint16_t speed)
    {
        std_msgs::msg::UInt16 msg;
        msg.data = speed;
        speed_pub_->publish(msg);
    }

    void publishPosition(uint16_t position)
    {
        std_msgs::msg::UInt16 msg;
        msg.data = position;
        position_pub_->publish(msg);
    }

    int readIntFromConsole(const std::string &prompt)
    {
        int value = 0;

        std::cout << prompt;
        std::cin >> value;

        if (!std::cin)
        {
            clearInput();
            throw std::runtime_error("Invalid integer input.");
        }

        return value;
    }

    void clearInput()
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr force_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr position_pub_;

    int actuator_id_{0};

    int position_min_{0};
    int position_max_{4095};
    int initial_position_{2700};
    int safe_position_{2700};

    int speed_min_{0};
    int speed_max_{1023};
    int initial_speed_{600};

    int operating_rate_limit_{500};

    bool force_enabled_{false};
    uint16_t current_speed_{600};
    uint16_t current_position_{2700};

    bool force_msg_dirty_{false};
    bool speed_msg_dirty_{false};
    bool position_msg_dirty_{false};

    std::mutex mutex_;
    std::thread pub_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MightyZapTestNode>();

    std::thread input_thread([&]()
                             {
                                 try
                                 {
                                     node->run();
                                 }
                                 catch (const std::exception &e)
                                 {
                                     RCLCPP_ERROR(node->get_logger(), "Test node input loop error: %s", e.what());
                                 } });

    rclcpp::spin(node);
    rclcpp::shutdown();

    if (input_thread.joinable())
        input_thread.join();

    return 0;
}