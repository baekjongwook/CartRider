#include <rclcpp/rclcpp.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>

class VescTestNode : public rclcpp::Node
{
public:
  VescTestNode() : Node("vesc_test_node")
  {
    if (!initCAN("can0"))
    {
      RCLCPP_FATAL(this->get_logger(), "CAN init failed");
      throw std::runtime_error("CAN init failed");
    }

    RCLCPP_INFO(this->get_logger(), "VESC Test Node Started (RPM Mode)");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&VescTestNode::loop, this)
    );

    input_thread_ = std::thread(&VescTestNode::inputLoop, this);
  }

  ~VescTestNode()
  {
    running_ = false;

    if (input_thread_.joinable())
      input_thread_.join();

    close(socket_);
  }

private:

  bool initCAN(const std::string& interface)
  {
    struct ifreq ifr;
    struct sockaddr_can addr;

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) return false;

    std::strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
      return false;

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      return false;

    return true;
  }

  void sendRPM(int id, int32_t erpm)
  {
    struct can_frame frame;

    frame.can_id = ((0x03 << 8) | id) | CAN_EFF_FLAG; 
    frame.can_dlc = 4;

    frame.data[0] = (erpm >> 24) & 0xFF;
    frame.data[1] = (erpm >> 16) & 0xFF;
    frame.data[2] = (erpm >> 8) & 0xFF;
    frame.data[3] = erpm & 0xFF;

    write(socket_, &frame, sizeof(frame));
  }

  void loop()
  {
    int32_t erpm_copy;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      erpm_copy = erpm_;
    }

    sendRPM(1, erpm_copy); 
  }

  void inputLoop()
  {
    while (running_)
    {
      float wheel_rpm;

      std::cout << "\nEnter wheel RPM (-100 ~ 100): ";
      std::cin >> wheel_rpm;

      if (!std::cin)
      {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input\n";
        continue;
      }

      wheel_rpm = std::clamp(wheel_rpm, -100.0f, 100.0f);

      int32_t erpm = static_cast<int32_t>(wheel_rpm * 140.0f);

      {
        std::lock_guard<std::mutex> lock(mutex_);
        erpm_ = erpm;
      }

      std::cout << "Set wheel RPM: " << wheel_rpm << " → ERPM: " << erpm << std::endl;
    }
  }

  int socket_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::thread input_thread_;
  std::mutex mutex_;

  int32_t erpm_ = 0;
  std::atomic<bool> running_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VescTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}