// Copyright 2018 Geoffrey Biggs

#if !defined DISPLAYER_ROS2_STYLE__DISPLAYER_NODE_HPP_
#define DISPLAYER_ROS2_STYLE__DISPLAYER_NODE_HPP_

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
#include <greeter_custom_msg/msg/greeting.hpp>

namespace displayer
{

class Displayer : public rclcpp::Node
{
public:
  DISPLAYER_PUBLIC
  Displayer();

private:
  rclcpp::Subscription<greeter_custom_msg::msg::Greeting>::SharedPtr sub_;

  void display_greeting(const greeter_custom_msg::msg::Greeting::SharedPtr msg);

};

} // namespace displayer

#endif // DISPLAYER_ROS2_STYLE__DISPLAYER_NODE_HPP_
