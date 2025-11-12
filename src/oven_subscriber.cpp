/**
 * @file oven_subscriber.cpp
 * @brief Subscriber node that receives oven status messages and logs them.
 *
 * Subscribes to the "oven_status" topic and prints received messages to the
 * node logger. Based on ROS2 Humble tutorials.
 */

// This code is based on ros2 humble tutorials from: https://docs.ros.org/en/humble/Tutorials.html

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class OvenSubscriber
 * @brief ROS2 node subscribing to oven_status topic and logging messages.
 */
class OvenSubscriber : public rclcpp::Node
{
 public:
  /**
   * @brief Construct a new OvenSubscriber node and create subscription.
   */
  OvenSubscriber() : Node("oven_subscriber")
  {
    using std::placeholders::_1;
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "oven_status", 10, std::bind(&OvenSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback for incoming oven status messages.
   * @param msg received string message
   */
  void topic_callback(const std_msgs::msg::String& msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Oven says: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Program entrypoint for the oven_subscriber node.
 *
 * Initializes ROS, creates the node, spins it, and shuts down cleanly.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OvenSubscriber>());
  rclcpp::shutdown();
  return 0;
}