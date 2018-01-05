// Copyright 2018 Geoffrey Biggs

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <greeting_service/srv/request_greeting.hpp>

using namespace std::chrono_literals;
using RequestGreeting = greeting_service::srv::RequestGreeting;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("greeting_client");

  // Create the service client
  auto client = node->create_client<RequestGreeting>("request_greeting");
  // Create a request to send the server
  auto request = std::make_shared<RequestGreeting::Request>();
  request->name = "Bob";

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
  }

  // Send the request to the server (this is asynchronous)
  auto result = client->async_send_request(request);
  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Received greeting: '%s'", result.get()->greeting.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Problem while waiting for response.");
  }

  // Shut down ROS
  rclcpp::shutdown();
  return 0;
}
