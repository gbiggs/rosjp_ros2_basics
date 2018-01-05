// Copyright 2018 Geoffrey Biggs

#include "greeter_custom_msg/greeter_component.hpp"
#include "greeter_custom_msg/msg/greeting.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <class_loader/class_loader_register_macro.h>
#include <rosidl_generator_cpp/message_initialization.hpp>

using namespace std::chrono_literals;

namespace greeter_ros2_style
{

Greeter::Greeter()
: Node("greeter")
{
  pub_ = create_publisher<greeter_custom_msg::msg::Greeting>("greeting");

  timer_ = create_wall_timer(1s, std::bind(&Greeter::broadcast_greeting, this));

  greeting_ = std::make_shared<greeter_custom_msg::msg::Greeting>(rosidl_generator_cpp::MessageInitialization::ZERO);
  greeting_->hello_text = "hello";
  greeting_->world_name = "world";
  greeting_->count = 0;
}

void Greeter::broadcast_greeting()
{
  RCLCPP_INFO(
      this->get_logger(),
      "Publishing greeting '%s %s %d'",
      greeting_->hello_text.c_str(),
      greeting_->world_name.c_str(),
      greeting_->count);
  pub_->publish(greeting_);
  ++greeting_->count;
}

} // namespace greeter_ros2_style

CLASS_LOADER_REGISTER_CLASS(greeter_ros2_style::Greeter, rclcpp::Node)
