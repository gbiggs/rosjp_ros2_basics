#include "displayer/displayer_component.hpp"

#include <class_loader/class_loader_register_macro.h>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace displayer
{

Displayer::Displayer()
: Node("displayer")
{
  sub_ = this->create_subscription<std_msgs::msg::String>(
    "greeting", std::bind(&Displayer::display_greeting, this, _1));
}

void Displayer::display_greeting(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received greeting '%s'", msg->data.c_str());
}

} // namespace displayer

CLASS_LOADER_REGISTER_CLASS(displayer::Displayer, rclcpp::Node)
