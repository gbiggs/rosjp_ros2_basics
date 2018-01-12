// Copyright 2018 Geoffrey Biggs, AIST
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

#if !defined MANAGED_GREETER__MANAGED_GREETER_COMPONENT_HPP_
#define MANAGED_GREETER__MANAGED_GREETER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MANAGED_GREETER_EXPORT __attribute__ ((dllexport))
    #define MANAGED_GREETER_IMPORT __attribute__ ((dllimport))
  #else
    #define MANAGED_GREETER_EXPORT __declspec(dllexport)
    #define MANAGED_GREETER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MANAGED_GREETER_BUILDING_DLL
    #define MANAGED_GREETER_PUBLIC MANAGED_GREETER_EXPORT
  #else
    #define MANAGED_GREETER_PUBLIC MANAGED_GREETER_IMPORT
  #endif
  #define MANAGED_GREETER_PUBLIC_TYPE MANAGED_GREETER_PUBLIC
  #define MANAGED_GREETER_LOCAL
#else
  #define MANAGED_GREETER_EXPORT __attribute__ ((visibility("default")))
  #define MANAGED_GREETER_IMPORT
  #if __GNUC__ >= 4
    #define MANAGED_GREETER_PUBLIC __attribute__ ((visibility("default")))
    #define MANAGED_GREETER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MANAGED_GREETER_PUBLIC
    #define MANAGED_GREETER_LOCAL
  #endif
  #define MANAGED_GREETER_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <greeter_custom_msg/msg/greeting.hpp>

namespace managed_greeter
{

class Greeter : public rclcpp_lifecycle::LifecycleNode
{
public:
  MANAGED_GREETER_PUBLIC
  Greeter();

private:
  rclcpp_lifecycle::LifecyclePublisher<greeter_custom_msg::msg::Greeting>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  greeter_custom_msg::msg::Greeting::SharedPtr greeting_;

  rcl_lifecycle_transition_key_t on_configure(const rclcpp_lifecycle::State &);
  rcl_lifecycle_transition_key_t on_activate(const rclcpp_lifecycle::State &);
  rcl_lifecycle_transition_key_t on_deactivate(const rclcpp_lifecycle::State &);
  rcl_lifecycle_transition_key_t on_cleanup(const rclcpp_lifecycle::State &);

  void broadcast_greeting();
};

} // namespace managed_greeter

#endif // MANAGED_GREETER__MANAGED_GREETER_COMPONENT_HPP_
