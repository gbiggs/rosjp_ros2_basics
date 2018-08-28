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

#include "greeter/greeter_component.hpp"

#include <class_loader/register_macro.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace greeter
{

Greeter::Greeter()
: Node("greeter")
{
  pub_ = create_publisher<greeting_msg::msg::Greeting>("greeting");

  timer_ = create_wall_timer(1s, std::bind(&Greeter::broadcast_greeting, this));

  greeting_ = std::make_shared<greeting_msg::msg::Greeting>(rosidl_generator_cpp::MessageInitialization::ZERO);
  greeting_->hello_text = "hello";
  greeting_->world_name = "world";
}

void Greeter::broadcast_greeting()
{
  RCLCPP_INFO(
      this->get_logger(),
      "Publishing greeting '%s %s %d'",
      greeting_->hello_text.c_str(),
      greeting_->world_name.c_str(),
      greeting_->count);
  pub_->publish(greeting_);
  ++greeting_->count;
}

} // namespace greeter

CLASS_LOADER_REGISTER_CLASS(greeter::Greeter, rclcpp::Node)
