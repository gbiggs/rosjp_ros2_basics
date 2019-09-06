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

#if !defined DISPLAYER__DISPLAYER_COMPONENT_HPP_
#define DISPLAYER__DISPLAYER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DISPLAYER_EXPORT __attribute__ ((dllexport))
    #define DISPLAYER_IMPORT __attribute__ ((dllimport))
  #else
    #define DISPLAYER_EXPORT __declspec(dllexport)
    #define DISPLAYER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DISPLAYER_BUILDING_DLL
    #define DISPLAYER_PUBLIC DISPLAYER_EXPORT
  #else
    #define DISPLAYER_PUBLIC DISPLAYER_IMPORT
  #endif
  #define DISPLAYER_PUBLIC_TYPE DISPLAYER_PUBLIC
  #define DISPLAYER_LOCAL
#else
  #define DISPLAYER_EXPORT __attribute__ ((visibility("default")))
  #define DISPLAYER_IMPORT
  #if __GNUC__ >= 4
    #define DISPLAYER_PUBLIC __attribute__ ((visibility("default")))
    #define DISPLAYER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DISPLAYER_PUBLIC
    #define DISPLAYER_LOCAL
  #endif
  #define DISPLAYER_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <greeting_msg/msg/greeting.hpp>

namespace displayer_custom_msg
{

class Displayer : public rclcpp::Node
{
public:
  DISPLAYER_PUBLIC explicit Displayer(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<greeting_msg::msg::Greeting>::SharedPtr sub_;

  void display_greeting(const greeting_msg::msg::Greeting::SharedPtr msg);
};

} // namespace displayer_custom_msg

#endif // DISPLAYER__DISPLAYER_COMPONENT_HPP_
