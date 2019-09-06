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

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <greeter/greeter_component.hpp>
#include <displayer/displayer_component.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // タイマーコールバック、トピックコールバック等を行うexecutor
  rclcpp::executors::SingleThreadedExecutor exec;

  // Greeterコンポーネントノードのインスタンスを作成しexecutorに登録する
  auto greeter = std::make_shared<greeter_custom_msg::Greeter>(options);
  exec.add_node(greeter);
  // Displayerコンポーネントノードのインスタンスを作成しexecutorに登録する
  auto displayer = std::make_shared<displayer_custom_msg::Displayer>(options);
  exec.add_node(displayer);

  // Ctrl-Cが押されるまでexecutorを実行する
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
