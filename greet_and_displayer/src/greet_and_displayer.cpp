// Copyright 2018 Geoffrey Biggs

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <greeter_custom_msg/greeter_component.hpp>
#include <displayer/displayer_component.hpp>

int main(int argc, char *argv[]) {
  // Initialise the ROS infrastructure
  rclcpp::init(argc, argv);

  // Executor responsible for executing timer callbacks, topic callbacks, etc.
  rclcpp::executors::SingleThreadedExecutor exec;

  // Create an instance of the Greeter component and add it to the executor
  auto greeter = std::make_shared<greeter_custom_msg::Greeter>();
  exec.add_node(greeter);
  // Create an instance of the Displayer component and add it to the executor
  auto displayer = std::make_shared<displayer::Displayer>();
  exec.add_node(displayer);

  // Run the executor until Ctrl-C is pressed
  exec.spin();

  // Shut down ROS
  rclcpp::shutdown();
  return 0;
}
