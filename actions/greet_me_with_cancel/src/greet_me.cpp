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
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <greeting_actions/action/process_greeting.hpp>

using namespace std::chrono_literals;
using ProcessGreeting = greeting_actions::action::ProcessGreeting;

void feedback_callback(
  std::shared_ptr<rclcpp::Node> node,
  rclcpp_action::ClientGoalHandle<ProcessGreeting>::SharedPtr,
  const std::shared_ptr<const ProcessGreeting::Feedback> feedback)
{
  RCLCPP_INFO(node->get_logger(), "Greeting creation %d%% complete", feedback->percent_complete);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("greet_me");

  // アクションクライントを作成する
  auto client = rclcpp_action::create_client<ProcessGreeting>(node, "process_greeting");

  // アクションが提供されているまでに待つ
  while (!client->wait_for_action_server(1s)) {
    // シャットダウンされたかどうか確認する
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for action server.");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for action server...");
  }

  // アクションGoalを作成する
  auto goal = ProcessGreeting::Goal();
  if (argc > 1) {
    goal.name = std::string(argv[1]);
  } else {
    goal.name = std::string("Bill");
  }

  RCLCPP_INFO(node->get_logger(), "Sending goal: '%s'", goal.name.c_str());
  using std::placeholders::_1;
  using std::placeholders::_2;
  auto send_goal_options = rclcpp_action::Client<ProcessGreeting>::SendGoalOptions();
  send_goal_options.feedback_callback = std::bind(&feedback_callback, node, _1, _2);
  auto goal_handle_future = client->async_send_goal(goal, send_goal_options);

  // Goalがサーバーでアクセプトされるまでに待つ
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Send goal call failed");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp_action::ClientGoalHandle<ProcessGreeting>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  auto result_future = client->async_get_result(goal_handle);
  auto wait_result = rclcpp::spin_until_future_complete(node, result_future, 5s);
  if (wait_result == rclcpp::executor::FutureReturnCode::TIMEOUT) {
    RCLCPP_INFO(node->get_logger(), "Cancelling goal");
    auto cancel_result_future = client->async_cancel_goal(goal_handle);
    if (rclcpp::spin_until_future_complete(node, cancel_result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to cancel goal");
      rclcpp::shutdown();
      return 1;
    }
  } else if (wait_result != rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get action result");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for result again");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get action result");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp_action::ClientGoalHandle<ProcessGreeting>::WrappedResult result = result_future.get();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      rclcpp::shutdown();
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was cancelled");
      rclcpp::shutdown();
      return 1;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown action result code");
      rclcpp::shutdown();
      return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Received action result: '%s'", result.result->greeting.c_str());

  rclcpp::shutdown();
  return 0;
}
