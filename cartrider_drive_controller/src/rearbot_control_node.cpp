#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"
#include "cartrider_drive_controller/differential_drive.hpp"

class RearbotControlNode : public rclcpp::Node
{
public:
  RearbotControlNode()
      : Node("rearbot_control_node")
  {
    r_ = this->declare_parameter<double>("rear_wheel_radius");
    L_ = this->declare_parameter<double>("rear_track_width");

    rmd_motor_ids_ =
        this->declare_parameter<std::vector<int64_t>>("rear_rmd_motor_ids", std::vector<int64_t>{});

    if (rmd_motor_ids_.size() != 2)
    {
      RCLCPP_FATAL(
          this->get_logger(),
          "Parameter 'rear_rmd_motor_ids' must contain exactly 2 elements. Got %zu",
          rmd_motor_ids_.size());
      throw std::runtime_error("Invalid rear_rmd_motor_ids size");
    }

    left_id_ = static_cast<int>(rmd_motor_ids_[0]);
    right_id_ = static_cast<int>(rmd_motor_ids_[1]);

    diff_ = std::make_unique<vehicle_kinematics::DifferentialDrive>(r_, L_);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&RearbotControlNode::cmdCallback, this, std::placeholders::_1));

    cmd_joy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_joy",
        10,
        std::bind(&RearbotControlNode::cmdJoyCallback, this, std::placeholders::_1));

    joy_sig_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "joy_control_sig",
        10,
        std::bind(&RearbotControlNode::joySigCallback, this, std::placeholders::_1));

    docking_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/docking_state",
        10,
        std::bind(&RearbotControlNode::dockingStateCallback, this, std::placeholders::_1));

    command_pub_ = this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>(
        "rmd_command",
        10);

    RCLCPP_INFO(
        this->get_logger(),
        "Rearbot Control Node Started. Input Mode: JOYSTICK, rear_wheel_radius=%.4f, rear_track_width=%.4f",
        r_,
        L_);
  }

private:
  bool joy_mode_active_{true};
  bool docking_state_{false};

  std::unique_ptr<vehicle_kinematics::DifferentialDrive> diff_;

  double r_{0.0};
  double L_{0.0};

  std::vector<int64_t> rmd_motor_ids_;

  int left_id_{0};
  int right_id_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sig_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_state_sub_;

  rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr command_pub_;

private:
  void joySigCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (joy_mode_active_ == msg->data)
    {
      return;
    }

    joy_mode_active_ = msg->data;

    RCLCPP_INFO(
        this->get_logger(),
        "Control Input Mode Switched: [%s]",
        joy_mode_active_ ? "JOYSTICK" : "NAVIGATION");
  }

  void dockingStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (docking_state_ == msg->data)
    {
      return;
    }

    docking_state_ = msg->data;

    RCLCPP_INFO(
        this->get_logger(),
        "Docking State Changed: %s",
        docking_state_ ? "DOCKED - rearbot independent controller disabled"
                       : "UNDOCKED - rearbot independent controller enabled");
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (joy_mode_active_)
    {
      return;
    }

    handleCommand(msg, "NAV");
  }

  void cmdJoyCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!joy_mode_active_)
    {
      return;
    }

    handleCommand(msg, "JOY");
  }

  void handleCommand(
      const geometry_msgs::msg::Twist::SharedPtr msg,
      const std::string &source)
  {
    if (docking_state_)
    {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "[%s] Docked state: rearbot independent cmd_vel ignored. Rear wheels are controlled by frontbot 2WS-4WD controller.",
          source.c_str());
      return;
    }

    const auto output = diff_->compute(msg->linear.x, msg->angular.z);

    const double left_radps = -output.left_w;
    const double right_radps = output.right_w;

    cartrider_rmd_sdk::msg::MotorCommandArray cmd;

    cartrider_rmd_sdk::msg::MotorCommand left_cmd;
    left_cmd.id = left_id_;
    left_cmd.target = left_radps;

    cartrider_rmd_sdk::msg::MotorCommand right_cmd;
    right_cmd.id = right_id_;
    right_cmd.target = right_radps;

    cmd.commands.push_back(left_cmd);
    cmd.commands.push_back(right_cmd);

    command_pub_->publish(cmd);

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][DIFF] v=%.3f w=%.3f -> left=%.3f right=%.3f",
        source.c_str(),
        msg->linear.x,
        msg->angular.z,
        left_radps,
        right_radps);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearbotControlNode>());
  rclcpp::shutdown();
  return 0;
}