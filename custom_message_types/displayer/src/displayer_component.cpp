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

#include "displayer/displayer_component.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace displayer_custom_msg
{

Displayer::Displayer(const rclcpp::NodeOptions & options)
: Node("displayer", options)
{
  sub_ = this->create_subscription<greeting_msg::msg::Greeting>(
    "greeting", 10, std::bind(&Displayer::display_greeting, this, _1));
}

void Displayer::display_greeting(const greeting_msg::msg::Greeting::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received greeting '%s %s %d'",
    msg->hello_text.c_str(), msg->world_name.c_str(), msg->count);
}

} // namespace displayer_custom_msg

#include "rclcpp_components/register_node_macro.hpp"

// 動的にコンポーネントノードをロードできるために登録する
RCLCPP_COMPONENTS_REGISTER_NODE(displayer_custom_msg::Displayer)
