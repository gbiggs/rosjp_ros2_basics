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

#if !defined GREETING_SERVER__GREETING_SERVER_COMPONENT_HPP_
#define GREETING_SERVER__GREETING_SERVER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GREETING_SERVER_EXPORT __attribute__ ((dllexport))
    #define GREETING_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define GREETING_SERVER_EXPORT __declspec(dllexport)
    #define GREETING_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef GREETING_SERVER_BUILDING_DLL
    #define GREETING_SERVER_PUBLIC GREETING_SERVER_EXPORT
  #else
    #define GREETING_SERVER_PUBLIC GREETING_SERVER_IMPORT
  #endif
  #define GREETING_SERVER_PUBLIC_TYPE GREETING_SERVER_PUBLIC
  #define GREETING_SERVER_LOCAL
#else
  #define GREETING_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define GREETING_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define GREETING_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define GREETING_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GREETING_SERVER_PUBLIC
    #define GREETING_SERVER_LOCAL
  #endif
  #define GREETING_SERVER_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <request_greeting_service/srv/request_greeting.hpp>
#include <memory>

namespace greeting_server
{

using RequestGreeting = request_greeting_service::srv::RequestGreeting;

class GreetingServer : public rclcpp::Node
{
public:
  GREETING_SERVER_PUBLIC explicit GreetingServer(const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<RequestGreeting>::SharedPtr server_;

  void send_greeting(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<RequestGreeting::Request> request,
    const std::shared_ptr<RequestGreeting::Response> response);
};

} // namespace greeting_server

#endif // GREETING_SERVER__GREETING_SERVER_COMPONENT_HPP_
