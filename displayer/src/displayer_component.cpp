// Copyright 2018 Geoffrey Biggs

#include "displayer/displayer_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <greeter_custom_msg/msg/greeting.hpp>
#include <class_loader/class_loader_register_macro.h>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace displayer
{

Displayer::Displayer()
: Node("displayer")
{
  sub_ = this->create_subscription<greeter_custom_msg::msg::Greeting>(
    "greeting", std::bind(&Displayer::display_greeting, this, _1));
}

void Displayer::display_greeting(const greeter_custom_msg::msg::Greeting::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received greeting '%s %s %d'",
    msg->hello_text.c_str(), msg->world_name.c_str(), msg->count);
}

} // namespace displayer

CLASS_LOADER_REGISTER_CLASS(displayer::Displayer, rclcpp::Node)
