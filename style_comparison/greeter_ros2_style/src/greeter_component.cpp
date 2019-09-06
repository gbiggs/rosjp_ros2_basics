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

#include "greeter_ros2_style/greeter_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace greeter_ros2_style
{

Greeter::Greeter(const rclcpp::NodeOptions & options)
: Node("greeter", options)
{
  pub_ = create_publisher<std_msgs::msg::String>("greeting", 10);

  // ノードの振る舞えを定期的に実行するためにタイマーを利用する
  timer_ = create_wall_timer(1s, std::bind(&Greeter::broadcast_greeting, this));
}

// ノードの振る舞え
// タイマーイベント毎に実行される
void Greeter::broadcast_greeting()
{
  auto greeting = std::make_shared<std_msgs::msg::String>();
  greeting->data = "hello world";
  RCLCPP_INFO(this->get_logger(), "Publishing greeting '%s'", greeting->data.c_str());
  pub_->publish(*greeting);
}

} // namespace greeter_ros2_style

#include "rclcpp_components/register_node_macro.hpp"

// 動的にコンポーネントノードをロードできるために登録する
RCLCPP_COMPONENTS_REGISTER_NODE(greeter_ros2_style::Greeter)
