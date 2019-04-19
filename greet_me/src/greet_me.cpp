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
  auto goal_handle_future = client->async_send_goal(
    goal,
    std::bind(&feedback_callback, node, _1, _2));

  // Goalがサーバーでアクセプトされるまでに待つ
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Send goal call failed");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<ProcessGreeting>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // サーバーでアクションの実行が終わるまでに待つ
  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  auto result_future = goal_handle->async_result();
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get action result");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<ProcessGreeting>::Result result = result_future.get();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was cancelled");
      return 1;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown action result code");
      return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Received action result: '%s'", result.response->greeting.c_str());

  rclcpp::shutdown();
  return 0;
}
