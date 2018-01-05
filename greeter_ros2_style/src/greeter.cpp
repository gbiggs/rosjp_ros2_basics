// Copyright 2018 Geoffrey Biggs

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "greeter_ros2_style/greeter_component.hpp"

int main(int argc, char *argv[]) {
  // Initialise the ROS infrastructure
  rclcpp::init(argc, argv);
  // Create an instance of the component and spin on it
  rclcpp::spin(std::make_shared<greeter_ros2_style::Greeter>());
  // Shut down ROS
  rclcpp::shutdown();
  return 0;
}
