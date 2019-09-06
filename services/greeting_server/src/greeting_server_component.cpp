// Copyright 2019 Geoffrey Biggs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "greeting_server/greeting_server_component.hpp"

namespace greeting_server
{

GreetingServer::GreetingServer(const rclcpp::NodeOptions & options)
: Node("greeting_server", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  server_ = this->create_service<RequestGreeting>(
    "request_greeting",
    std::bind(&GreetingServer::send_greeting, this, _1, _2, _3));
}

void GreetingServer::send_greeting(
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

} // namespace greeting_server

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(greeting_server::GreetingServer)
