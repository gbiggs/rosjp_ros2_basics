// Copyright 2018 Geoffrey Biggs

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
