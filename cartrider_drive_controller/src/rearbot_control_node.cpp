// Rearbot Control Node
// 2026.02.18 백종욱

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"

#include "cartrider_drive_controller/differential_drive.hpp"

class RearbotControlNode : public rclcpp::Node
{
public:
  RearbotControlNode()
  : Node("rearbot_control_node")
  {
    r_ = this->declare_parameter<double>("wheel_radius");
    L_ = this->declare_parameter<double>("wheel_separation");
    max_w_ = this->declare_parameter<double>("max_wheel_w");

    left_id_ = this->declare_parameter<int>("left_motor_id");
    right_id_ = this->declare_parameter<int>("right_motor_id");

    diff_ = std::make_unique<vehicle_kinematics::DifferentialDrive>(r_, L_, max_w_);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 
        10,
        std::bind(&RearbotControlNode::cmdCallback, this, std::placeholders::_1));

    command_pub_ = create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>(
        "rmd_command", 
        10);
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto output = diff_->compute(msg->linear.x, msg->angular.z);

    double left_rpm = -output.left_w * 60.0 / (2.0 * M_PI);
    double right_rpm = output.right_w * 60.0 / (2.0 * M_PI);

    RCLCPP_INFO(this->get_logger(),
              "cmd_vel: v=%.3f  w=%.3f  ->  left RPM=%.3f  right RPM=%.3f",
              msg->linear.x,
              msg->angular.z,
              left_rpm,
              right_rpm);


    cartrider_rmd_sdk::msg::MotorCommandArray cmd;

    cartrider_rmd_sdk::msg::MotorCommand left_cmd;
    left_cmd.id = left_id_;
    left_cmd.target = -output.left_w;

    cartrider_rmd_sdk::msg::MotorCommand right_cmd;
    right_cmd.id = right_id_;
    right_cmd.target = output.right_w;

    cmd.commands.push_back(left_cmd);
    cmd.commands.push_back(right_cmd);

    command_pub_->publish(cmd);
  }

  std::unique_ptr<vehicle_kinematics::DifferentialDrive> diff_;

  double r_;
  double L_;
  double max_w_;

  int left_id_;  
  int right_id_;  

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_; 
  rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr command_pub_; 
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearbotControlNode>());
  rclcpp::shutdown();
  return 0;
}