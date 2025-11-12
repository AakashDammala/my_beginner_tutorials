/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Aakash
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

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