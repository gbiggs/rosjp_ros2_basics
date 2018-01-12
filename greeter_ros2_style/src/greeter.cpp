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

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "greeter_ros2_style/greeter_component.hpp"

int main(int argc, char *argv[]) {
  // ROSをイニシャライズする
  rclcpp::init(argc, argv);
  // コンポーネントノードのインスタンスを作成する
  auto greeter = std::make_shared<greeter_ros2_style::Greeter>();
  // コンポーネントノードを実行する
  rclcpp::spin(greeter);
  // Shut down ROS
  rclcpp::shutdown();
  return 0;
}
