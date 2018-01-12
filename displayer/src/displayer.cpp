#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "displayer/displayer_component.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<displayer::Displayer>());
  rclcpp::shutdown();
  return 0;
}
