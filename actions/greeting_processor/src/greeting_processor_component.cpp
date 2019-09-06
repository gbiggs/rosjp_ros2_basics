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

#include "greeting_processor/greeting_processor_component.hpp"

namespace greeting_processor
{

GreetingProcessor::GreetingProcessor(const rclcpp::NodeOptions & options)
: Node("greeting_processor", options)
{
}

void GreetingProcessor::make_action_server()
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  server_ = rclcpp_action::create_server<ProcessGreeting>(
    this->shared_from_this(),
    "process_greeting",
    std::bind(&GreetingProcessor::handle_goal, this, _1, _2),
    std::bind(&GreetingProcessor::handle_cancel, this, _1),
    std::bind(&GreetingProcessor::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse GreetingProcessor::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ProcessGreeting::Goal> goal)
{
  RCLCPP_INFO(
    rclcpp::get_logger("server"),
    "Got request to process greeting from human '%s'",
    goal->name.c_str());
  (void)uuid;
  if (goal->name == "bob" || goal->name == "Bob") {
    // Reject goals asking to greet Bob
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GreetingProcessor::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessGreeting>> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
  (void)goal_handle;
  // Accept all cancel requests
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GreetingProcessor::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessGreeting>> goal_handle)
{
  using std::placeholders::_1;
  RCLCPP_INFO(rclcpp::get_logger("server"), "Beginning execution of goal");
  // Create a thread to handle the actual goal execution so we remain responsive to new goals
  std::thread{std::bind(&GreetingProcessor::execute, this, _1), goal_handle}.detach();
}

void GreetingProcessor::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessGreeting>> goal_handle)
{
  rclcpp::Rate rate(1);
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(
    rclcpp::get_logger("server"),
    "Executing goal for human '%s'",
    goal->name.c_str());
  auto feedback = std::make_shared<ProcessGreeting::Feedback>();
  auto result = std::make_shared<ProcessGreeting::Result>();

  // Simulate long processing time while remaining cancellable
  for (int i=0; i < 101 && rclcpp::ok(); i += 10) {
    if (goal_handle->is_canceling()) {
      result->greeting = "Cancelled greeting";
      goal_handle->canceled(result);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Goal cancelled");
      return;
    }

    // Publish feedback on how processing is going
    feedback->percent_complete = i;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("server"), "%d complete", i);

    rate.sleep();
  }

  if (rclcpp::ok()) {
    result->greeting = "Hello, " + goal->name;
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal succeeded");
  }
}

} // namespace greeting_processor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(greeting_processor::GreetingProcessor)
