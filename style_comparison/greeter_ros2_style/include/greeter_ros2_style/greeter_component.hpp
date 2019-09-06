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

#if !defined GREETER_ROS2_STYLE__GREETER_NODE_HPP_
#define GREETER_ROS2_STYLE__GREETER_NODE_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GREETER_EXPORT __attribute__ ((dllexport))
    #define GREETER_IMPORT __attribute__ ((dllimport))
  #else
    #define GREETER_EXPORT __declspec(dllexport)
    #define GREETER_IMPORT __declspec(dllimport)
  #endif
  #ifdef GREETER_BUILDING_DLL
    #define GREETER_PUBLIC GREETER_EXPORT
  #else
    #define GREETER_PUBLIC GREETER_IMPORT
  #endif
  #define GREETER_PUBLIC_TYPE GREETER_PUBLIC
  #define GREETER_LOCAL
#else
  #define GREETER_EXPORT __attribute__ ((visibility("default")))
  #define GREETER_IMPORT
  #if __GNUC__ >= 4
    #define GREETER_PUBLIC __attribute__ ((visibility("default")))
    #define GREETER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GREETER_PUBLIC
    #define GREETER_LOCAL
  #endif
  #define GREETER_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace greeter_ros2_style
{

// ノードを実装するクラス
class Greeter : public rclcpp::Node
{
public:
  // クラスのシンボルをエクスポートする
  GREETER_PUBLIC explicit Greeter(const rclcpp::NodeOptions & options);

private:
  // パブリッシャーとタイマーを持つ変数
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ノードの振る舞えを実装するメンバー関数
  void broadcast_greeting();
};

} // namespace greeter_ros2_style

#endif // GREETER_ROS2_STYLE__GREETER_NODE_HPP_
