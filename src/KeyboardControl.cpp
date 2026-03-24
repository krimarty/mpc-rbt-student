#include <chrono>
#include <functional>
#include <fcntl.h>

#include "KeyboardControl.hpp"

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode(): rclcpp::Node("keyboard_control_node") {

    this->declare_parameter("robot_speed", 1.0);
    robot_speed_ = this->get_parameter("robot_speed").as_double();

    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto & param : params) {
                if (param.get_name() == "robot_speed") {
                    robot_speed_ = param.as_double();
                    RCLCPP_INFO(get_logger(), "robot_speed changed to %.2f", robot_speed_);
                }
            }
            return result;
        });

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardControlNode::timerCallback, this));

    // Set terminal settings to non-blocking
    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    
    RCLCPP_INFO(get_logger(), "Keyboard Control node started. Speed: %.2f", robot_speed_);
    RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");
}

KeyboardControlNode::~KeyboardControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    geometry_msgs::msg::Twist twist{};
    char c;

    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

    if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\033') { // ESC sequence (arrow keys)
                char seq[2];
                if (read(STDIN_FILENO, &seq, 2) != 2)
                    return;

                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A':
                            twist.linear.x = robot_speed_;  // up arrow
                            break;
                        case 'B':
                            twist.linear.x = -robot_speed_; // down arrow
                            break;
                        case 'C':
                            twist.angular.z = -robot_speed_; // right arrow
                            break;
                        case 'D':
                            twist.angular.z = robot_speed_;  // left arrow
                            break;
                    }
                }
            }

            twist_publisher_->publish(twist);
        }
    }
    // else no data was available, do nothing
}
