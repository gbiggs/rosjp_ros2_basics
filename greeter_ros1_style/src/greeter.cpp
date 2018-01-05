// Copyright 2018 Geoffrey Biggs

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

int main(int argc, char *argv[]) {
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("greeter");

  auto publisher = node->create_publisher<std_msgs::msg::String>("greeting");

  auto greeting = std::make_shared<std_msgs::msg::String>();
  greeting->data = "hello world";

  rclcpp::WallRate rate(1s);

  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Publishing greeting '%s'", greeting->data.c_str());
    publisher->publish(greeting);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
