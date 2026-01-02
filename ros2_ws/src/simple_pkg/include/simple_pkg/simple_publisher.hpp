#ifndef SIMPLE_PUBLISHER_HPP
#define SIMPLE_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimplePublisher : public rclcpp::Node {
private:
  void timer_callback();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  SimplePublisher(const std::string &name);
};

#endif //  SIMPLE_PUBLISHER_HPP