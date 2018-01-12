// Copyright 2018 Geoffrey Biggs

// ROS 2のrclcppライブラリをインクルードする
#include <rclcpp/rclcpp.hpp>
// std_msgs/Stringメッセージ型を利用する
#include <std_msgs/msg/string.hpp>
// 単位付け時間の利用のためにインクルードする
#include <chrono>
// std::make_shared等のためにインクルードする
#include <memory>

int main(int argc, char *argv[]) {
  // 単位付け時間の利用のために
  using namespace std::chrono_literals;

  // ROSのインフラストラクチャをイニシャライズする
  rclcpp::init(argc, argv);
  // ノードオブジェクトのインスタンスを作成する
  auto node = rclcpp::Node::make_shared("greeter");

  // データ送信のためにパブリッシャーを作成する
  // ここでトピックが作成される
  auto publisher = node->create_publisher<std_msgs::msg::String>("greeting");

  // 送信するデータを作成する
  auto greeting = std::make_shared<std_msgs::msg::String>();
  greeting->data = "hello world";

  // 周期を１秒にする
  rclcpp::WallRate rate(1s);

  // Ctrl-Cが押されるまでにループする
  while (rclcpp::ok()) {
    // ログを出力する
    RCLCPP_INFO(node->get_logger(), "Publishing greeting '%s'", greeting->data.c_str());
    // データ送信
    publisher->publish(greeting);
    // ノードのイベント等があれば実行する
    rclcpp::spin_some(node);
    // 次のループまで寝る
    rate.sleep();
  }
  // ROSのインフラストラクチャをシャットダウンする
  rclcpp::shutdown();
  return 0;
}
