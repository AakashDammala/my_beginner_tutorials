// This code is based on ros2 humble tutorials from: https://docs.ros.org/en/humble/Tutorials.html

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class OvenSubscriber : public rclcpp::Node
{
 public:
  OvenSubscriber() : Node("oven_subscriber")
  {
    using std::placeholders::_1;
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "oven_status", 10, std::bind(&OvenSubscriber::topic_callback, this, _1));
  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Oven says: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OvenSubscriber>());
  rclcpp::shutdown();
  return 0;
}