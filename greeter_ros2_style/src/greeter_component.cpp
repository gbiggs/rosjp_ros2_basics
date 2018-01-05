// Copyright 2018 Geoffrey Biggs

#include "greeter_ros2_style/greeter_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <class_loader/class_loader_register_macro.h>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace greeter_ros2_style
{

Greeter::Greeter()
: Node("greeter")
{
  pub_ = create_publisher<std_msgs::msg::String>("greeting");

  timer_ = create_wall_timer(1s, std::bind(&Greeter::broadcast_greeting, this));
}

void Greeter::broadcast_greeting()
{
  auto greeting = std::make_shared<std_msgs::msg::String>();
  greeting->data = "hello world";
  RCLCPP_INFO(this->get_logger(), "Publishing greeting '%s'", greeting->data.c_str());
  pub_->publish(greeting);
}

} // namespace greeter_ros2_style

CLASS_LOADER_REGISTER_CLASS(greeter_ros2_style::Greeter, rclcpp::Node)
