// This code is based on ros2 humble tutorials from: https://docs.ros.org/en/humble/Tutorials.html

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace std::chrono_literals;

class OvenPublisher : public rclcpp::Node
{
 public:
  OvenPublisher() : Node("oven_publisher"), time_rem_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("oven_status", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&OvenPublisher::timer_callback, this));
    RCLCPP_DEBUG_STREAM(this->get_logger(), "The oven has been powered up!");
  }

 private:
  void timer_callback()
  {
    if (time_rem_ <= 0)
    {
      std::string msg = "The Oven is now ready to use now";
      RCLCPP_INFO_STREAM(this->get_logger(), msg);

      std_msgs::msg::String pub_msg;
      pub_msg.data = msg;
      publisher_->publish(pub_msg);
    }
    else
    {
      int time_rem = 0;
      std::string msg =
          "The Oven is running, time remaining: " + std::to_string(time_rem) + " seconds";
      RCLCPP_INFO_STREAM(this->get_logger(), msg);

      std_msgs::msg::String pub_msg;
      pub_msg.data = msg;
      publisher_->publish(pub_msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int time_rem_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OvenPublisher>());
  rclcpp::shutdown();
  return 0;
}
