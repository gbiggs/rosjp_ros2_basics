// Copyright 2018 Geoffrey Biggs

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "greeting_service/srv/request_greeting.hpp"

using RequestGreeting = greeting_service::srv::RequestGreeting;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class GreetingService : public rclcpp::Node
{
public:
  GreetingService()
  : Node("greeting_service")
  {
    server_ = this->create_service<RequestGreeting>(
      "request_greeting",
      std::bind(&GreetingService::send_greeting, this, _1, _2, _3));
  }

private:
  void send_greeting(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<RequestGreeting::Request> request,
    const std::shared_ptr<RequestGreeting::Response> response)
  {
    (void)request_header;
    response->greeting = "Hello, " + request->name;
    RCLCPP_INFO(
      this->get_logger(),
      "Responding to greeting request with '%s'",
      response->greeting.c_str());
  }

  rclcpp::Service<RequestGreeting>::SharedPtr server_;
};

int main(int argc, char *argv[]) {
  // Initialise the ROS infrastructure
  rclcpp::init(argc, argv);
  // Create an instance of the component and spin on it
  rclcpp::spin(std::make_shared<GreetingService>());
  // Shut down ROS
  rclcpp::shutdown();
  return 0;
}
