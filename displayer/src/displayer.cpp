// Copyright 2018 Geoffrey Biggs

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "displayer/displayer_component.hpp"

int main(int argc, char *argv[]) {
  // Initialise the ROS infrastructure
  rclcpp::init(argc, argv);

  // Executor responsible for executing timer callbacks, topic callbacks, etc.
  rclcpp::executors::SingleThreadedExecutor exec;

  // Create an instance of the Displayer component and add it to the executor
  auto displayer = std::make_shared<displayer::Displayer>();
  exec.add_node(displayer);

  // Run the executor until Ctrl-C is pressed
  exec.spin();

  return 0;
}
