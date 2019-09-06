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

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <request_greeting_service/srv/request_greeting.hpp>

using namespace std::chrono_literals;
using RequestGreeting = request_greeting_service::srv::RequestGreeting;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("greeting_client");

  // サービスクライントを作成する
  auto client = node->create_client<RequestGreeting>("request_greeting");

  // サービスが提供されているまでに待つ
  while (!client->wait_for_service(1s)) {
    // シャットダウンされたかどうか確認する
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
  }

  // サーバーに送るリクエストを作成する
  auto request = std::make_shared<RequestGreeting::Request>();
  request->name = "Bob";
  // サーバーにリクエストを送る（非同期）
  auto result = client->async_send_request(request);
  // 結果を待つ
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Received greeting: '%s'", result.get()->greeting.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Problem while waiting for response.");
  }

  rclcpp::shutdown();
  return 0;
}
