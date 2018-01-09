// Copyright 2018 Geoffrey Biggs

#include "managed_greeter/managed_greeter_component.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <class_loader/class_loader_register_macro.h>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <greeter_custom_msg/msg/greeting.hpp>

using namespace std::chrono_literals;

namespace managed_greeter
{

Greeter::Greeter()
: rclcpp_lifecycle::LifecycleNode("greeter")
{
  // No construction necessary (node is uninitialised)
}

rcl_lifecycle_transition_key_t Greeter::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring node");

  // Initialise the node's data
  pub_ = create_publisher<greeter_custom_msg::msg::Greeting>("greeting");
  timer_ = create_wall_timer(1s, std::bind(&Greeter::broadcast_greeting, this));
  greeting_ = std::make_shared<greeter_custom_msg::msg::Greeting>(
    rosidl_generator_cpp::MessageInitialization::ZERO);
  greeting_->hello_text = "hello";
  greeting_->world_name = "world";
  greeting_->count = 0;

  // Inform the lifecycle manager that configuration succeeded
  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t Greeter::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating node");

  // Enable the publisher so messages will be sent
  pub_->on_activate();

  // Inform the lifecycle manager that the node activated successfully
  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t Greeter::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating node");

  // Disable the publisher so messages will *not* be sent
  pub_->on_deactivate();

  // Inform the lifecycle manager that the node deactivated successfully
  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t Greeter::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up node");

  pub_.reset();
  timer_.reset();
  greeting_.reset();

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
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

} // namespace managed_greeter

CLASS_LOADER_REGISTER_CLASS(managed_greeter::Greeter, rclcpp_lifecycle::LifecycleNode)
