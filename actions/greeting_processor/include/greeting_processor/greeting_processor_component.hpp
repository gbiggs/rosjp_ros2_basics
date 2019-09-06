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

#if !defined GREETING_PROCESSOR__GREETING_PROCESSOR_COMPONENT_HPP_
#define GREETING_PROCESSOR__GREETING_PROCESSOR_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GREETING_PROCESSOR_EXPORT __attribute__ ((dllexport))
    #define GREETING_PROCESSOR_IMPORT __attribute__ ((dllimport))
  #else
    #define GREETING_PROCESSOR_EXPORT __declspec(dllexport)
    #define GREETING_PROCESSOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef GREETING_PROCESSOR_BUILDING_DLL
    #define GREETING_PROCESSOR_PUBLIC GREETING_PROCESSOR_EXPORT
  #else
    #define GREETING_PROCESSOR_PUBLIC GREETING_PROCESSOR_IMPORT
  #endif
  #define GREETING_PROCESSOR_PUBLIC_TYPE GREETING_PROCESSOR_PUBLIC
  #define GREETING_PROCESSOR_LOCAL
#else
  #define GREETING_PROCESSOR_EXPORT __attribute__ ((visibility("default")))
  #define GREETING_PROCESSOR_IMPORT
  #if __GNUC__ >= 4
    #define GREETING_PROCESSOR_PUBLIC __attribute__ ((visibility("default")))
    #define GREETING_PROCESSOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GREETING_PROCESSOR_PUBLIC
    #define GREETING_PROCESSOR_LOCAL
  #endif
  #define GREETING_PROCESSOR_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <greeting_actions/action/process_greeting.hpp>
#include <memory>

namespace greeting_processor
{

using ProcessGreeting = greeting_actions::action::ProcessGreeting;

class GreetingProcessor : public rclcpp::Node
{
public:
  GREETING_PROCESSOR_PUBLIC explicit GreetingProcessor(const rclcpp::NodeOptions & options);

  void make_action_server();

private:
  rclcpp_action::Server<ProcessGreeting>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
    const std::array<uint8_t, 16> & uuid,
    std::shared_ptr<const ProcessGreeting::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessGreeting>> goal_handle);

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessGreeting>> goal_handle);

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessGreeting>> goal_handle);
};

} // namespace greeting_processor

#endif // GREETING_PROCESSOR__GREETING_PROCESSOR_COMPONENT_HPP_
