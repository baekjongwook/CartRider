// Teleop_keyboard
// 2026.03.05 백종욱

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <iostream>
#include <algorithm>

#define LIN_STEP 0.01
#define ANG_STEP 0.01

#define MAX_LIN 1.0
#define MAX_ANG 3.0

class Teleop : public rclcpp::Node
{
public:
    Teleop() : Node("teleop_keyboard")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        setupTerminal();
        printHelp();
    }

    ~Teleop()
    {
        restoreTerminal();
    }

    void spin()
    {
        while (rclcpp::ok())
        {
            char key = getKey();

            switch (key)
            {
                case 'w':
                    target_linear_ = std::min(target_linear_ + LIN_STEP, MAX_LIN);
                    break;
                case 'x':
                    target_linear_ = std::max(target_linear_ - LIN_STEP, -MAX_LIN);
                    break;
                case 'a':
                    target_angular_ = std::min(target_angular_ + ANG_STEP, MAX_ANG);
                    break;
                case 'd':
                    target_angular_ = std::max(target_angular_ - ANG_STEP, -MAX_ANG);
                    break;
                case ' ':
                case 's':
                    target_linear_ = 0.0;
                    target_angular_ = 0.0;
                    control_linear_ = 0.0;
                    control_angular_ = 0.0;
                    break;
                case 3:
                    return;
                default:
                    break;
            }

            control_linear_  = makeProfile(control_linear_, target_linear_, LIN_STEP/2.0);
            control_angular_ = makeProfile(control_angular_, target_angular_, ANG_STEP/2.0);

            publish();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    double target_linear_ = 0.0;
    double target_angular_ = 0.0;
    double control_linear_ = 0.0;
    double control_angular_ = 0.0;

    struct termios orig_termios_;

    void publish()
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = control_linear_;
        msg.angular.z = control_angular_;
        pub_->publish(msg);

        std::cout << "\rLinear: " << control_linear_ << "  Angular: " << control_angular_ << "   " << std::flush;
    }

    double makeProfile(double output, double input, double slop)
    {
        if (input > output)
            output = std::min(input, output + slop);
        else if (input < output)
            output = std::max(input, output - slop);
        return output;
    }

    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        struct termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

    char getKey()
    {
        fd_set set;
        struct timeval timeout;
        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);

        timeout.tv_sec = 0;
        timeout.tv_usec = 20000;

        int rv = select(STDIN_FILENO+1, &set, NULL, NULL, &timeout);

        if (rv > 0)
        {
            char c;
            read(STDIN_FILENO, &c, 1);
            return c;
        }
        return 0;
    }

    void printHelp()
    {
        std::cout <<
        "\nControl Your Robot\n"
        "---------------------------\n"
        "        w\n"
        "   a    s    d\n"
        "        x\n\n"
        "w/x : linear +/-\n"
        "a/d : angular +/-\n"
        "space or s : stop\n"
        "CTRL-C to quit\n\n";
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}