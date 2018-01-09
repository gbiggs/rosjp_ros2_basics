// Copyright 2018 Geoffrey Biggs

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "managed_greeter/managed_greeter_component.hpp"

int main(int argc, char *argv[]) {
  // Initialise the ROS infrastructure
  rclcpp::init(argc, argv);
  // Create an instance of the component and spin on it
  rclcpp::spin(std::make_shared<managed_greeter::Greeter>());
  // Shut down ROS
  rclcpp::shutdown();
}
