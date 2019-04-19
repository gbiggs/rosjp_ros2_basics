#include "greeting_processor/greeting_processor_component.hpp"

#include <class_loader/register_macro.hpp>

namespace greeting_processor
{

GreetingProcessor::GreetingProcessor()
: Node("greeting_processor")
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
  const std::array<uint8_t, 16> & uuid,
  std::shared_ptr<const ProcessGreeting::Goal> goal)
{
  RCLCPP_INFO(
    rclcpp::get_logger("server"),
    "Got request to process greeting from human '%s'",
    goal->name.c_str());
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
      goal_handle->set_canceled(result);
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
    goal_handle->set_succeeded(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal succeeded");
  }
}

} // namespace greeting_processor

CLASS_LOADER_REGISTER_CLASS(greeting_processor::GreetingProcessor, rclcpp::Node)
