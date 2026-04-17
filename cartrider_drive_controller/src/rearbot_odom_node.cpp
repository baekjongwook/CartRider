// Rearbot Odom Node
// 2026.03.05 백종욱

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cartrider_rmd_sdk/msg/motor_state_array.hpp>

class RearbotOdomNode : public rclcpp::Node
{
public:
  RearbotOdomNode()
      : Node("rearbot_odom_node"),
        x_(0.0), y_(0.0), theta_(0.0),
        initialized_(false)
  {
    r_ = this->declare_parameter<double>("wheel_radius");
    L_ = this->declare_parameter<double>("track_width");

    rmd_motor_ids_ =
        this->declare_parameter<std::vector<int64_t>>("rmd_motor_ids", std::vector<int64_t>{});

    if (rmd_motor_ids_.size() != 2)
    {
      RCLCPP_FATAL(
          this->get_logger(),
          "Parameter 'rmd_motor_ids' must contain exactly 2 elements. Got %zu",
          rmd_motor_ids_.size());
      throw std::runtime_error("Invalid rmd_motor_ids size");
    }

    left_id_ = static_cast<int>(rmd_motor_ids_[0]);
    right_id_ = static_cast<int>(rmd_motor_ids_[1]);

    sub_ = create_subscription<cartrider_rmd_sdk::msg::MotorStateArray>(
        "rmd_state",
        10,
        std::bind(&RearbotOdomNode::stateCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "odom",
        10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Rearbot Odom Node Started");
  }

private:
  void stateCallback(const cartrider_rmd_sdk::msg::MotorStateArray::SharedPtr msg)
  {
    double left_pos = 0.0;
    double right_pos = 0.0;

    bool left_found = false;
    bool right_found = false;

    for (const auto &s : msg->states)
    {
      if (s.id == left_id_)
      {
        left_pos = s.position;
        left_found = true;
      }
      else if (s.id == right_id_)
      {
        right_pos = s.position;
        right_found = true;
      }
    }

    if (!left_found || !right_found)
      return;

    rclcpp::Time now = this->now();

    if (!initialized_)
    {
      prev_left_ = left_pos;
      prev_right_ = right_pos;
      prev_time_ = now;
      initialized_ = true;
      return;
    }

    double dt = (now - prev_time_).seconds();
    if (dt <= 0.0)
      return;

    double dtheta_left = left_pos - prev_left_;
    double dtheta_right = right_pos - prev_right_;

    prev_left_ = left_pos;
    prev_right_ = right_pos;
    prev_time_ = now;

    double ds_left = -r_ * dtheta_left;
    double ds_right = r_ * dtheta_right;

    double ds = (ds_left + ds_right) / 2.0;
    double dtheta = (ds_right - ds_left) / L_;

    x_ += ds * std::cos(theta_ + dtheta / 2.0);
    y_ += ds * std::sin(theta_ + dtheta / 2.0);
    theta_ += dtheta;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    double vx = ds / dt;
    double vtheta = dtheta / dt;

    publishOdom(now, vx, vtheta);
    publishTF(now);
  }

  void publishOdom(rclcpp::Time stamp, double vx, double vtheta)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vtheta;

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[35] = 0.02;

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[35] = 0.02;

    odom_pub_->publish(odom);
  }

  void publishTF(rclcpp::Time stamp)
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
  }

  double r_;
  double L_;
  std::vector<int64_t> rmd_motor_ids_;
  int left_id_;
  int right_id_;

  double x_, y_, theta_;
  double prev_left_, prev_right_;
  rclcpp::Time prev_time_;
  bool initialized_;

  rclcpp::Subscription<cartrider_rmd_sdk::msg::MotorStateArray>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearbotOdomNode>());
  rclcpp::shutdown();
  return 0;
}