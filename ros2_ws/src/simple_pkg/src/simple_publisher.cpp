#include "simple_pkg/simple_publisher.hpp"

SimplePublisher::SimplePublisher(const std::string &name) : Node(name) {
  pub_ = create_publisher<std_msgs::msg::String>("hello_world_topic", 10);
  timer_ = create_wall_timer(std::chrono::seconds(1),
                             std::bind(&SimplePublisher::timer_callback, this));
  RCLCPP_INFO_STREAM(get_logger(), "Publishing at 1 Hz");
}

void SimplePublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Hello ROS 2";
  pub_->publish(message);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimplePublisher>("simple_publisher_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}