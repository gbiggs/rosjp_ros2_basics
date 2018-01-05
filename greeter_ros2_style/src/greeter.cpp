// Copyright 2018 Geoffrey Biggs

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "greeter_ros2_style/greeter_component.hpp"

int main(int argc, char *argv[]) {
  // Initialise the ROS infrastructure
  rclcpp::init(argc, argv);

  // Executor responsible for executing timer callbacks, topic callbacks, etc.
  rclcpp::executors::SingleThreadedExecutor exec;

  // Create an instance of the Greeter component and add it to the executor
  auto greeter = std::make_shared<greeter_ros2_style::Greeter>();
  exec.add_node(greeter);

  // Run the executor until Ctrl-C is pressed
  exec.spin();

  return 0;
}
