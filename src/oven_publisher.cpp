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
 * @file oven_publisher.cpp
 * @brief Publishes oven status and provides a service to add 5 seconds to runtime.
 *
 * This node publishes human-readable oven status messages on the "oven_status" topic
 * and provides a Trigger service named "add_5_sec" which extends the oven runtime.
 *
 * Based on ROS2 Humble tutorials: https://docs.ros.org/en/humble/Tutorials.html
 */

// This code is based on ros2 humble tutorials from: https://docs.ros.org/en/humble/Tutorials.html

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

using namespace std::chrono_literals;

/**
 * @class OvenPublisher
 * @brief ROS2 node that publishes oven status and offers a service to add time.
 *
 * The node declares a string parameter ("name") to identify the oven instance
 * in log and published messages.
 */
class OvenPublisher : public rclcpp::Node
{
 public:
  /**
   * @brief Construct a new OvenPublisher node.
   *
   * Declares the parameter `name` and initializes the publisher, service and timer.
   */
  OvenPublisher() : Node("oven_publisher")
  {
    // initialize the end time variable
    end_time_ = std::chrono::steady_clock::now();

    // start the oven status publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("oven_status", 10);

    // Variables required for the add 5 seconds service server
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    add_5_sec_service_ = this->create_service<std_srvs::srv::Trigger>(
        "add_5_sec", std::bind(&OvenPublisher::add_5_sec_cb, this, _1, _2, _3));

    // main timer
    timer_ = this->create_wall_timer(500ms, std::bind(&OvenPublisher::timer_callback, this));

    // declare and read the oven identifier parameter (string)
    oven_name_ = this->declare_parameter<std::string>("name", "default_oven");

    RCLCPP_DEBUG_STREAM(this->get_logger(), "The oven '" << oven_name_ << "' has been powered up!");
  }

 private:
  /**
   * @brief Timer callback that publishes current oven status periodically.
   *
   * Publishes a short string message that includes the oven name and remaining time
   * (or readiness) to the "oven_status" topic.
   */
  void timer_callback()
  {
    auto time_rem = end_time_ - std::chrono::steady_clock::now();

    if (time_rem < std::chrono::steady_clock::duration::zero())
    {
      std::string msg = "The Oven '" + oven_name_ + "' is now ready to use";
      RCLCPP_INFO_STREAM(this->get_logger(), msg);

      std_msgs::msg::String pub_msg;
      pub_msg.data = msg;
      publisher_->publish(pub_msg);
    }
    else
    {
      int time_rem_sec = std::chrono::duration_cast<std::chrono::seconds>(time_rem).count();

      std::string msg = "The Oven '" + oven_name_ +
                        "' is running, time remaining: " + std::to_string(time_rem_sec) +
                        " seconds";
      RCLCPP_INFO_STREAM(this->get_logger(), msg);

      std_msgs::msg::String pub_msg;
      pub_msg.data = msg;
      publisher_->publish(pub_msg);
    }
  }

  /**
   * @brief Service callback that adds 5 seconds to the oven runtime.
   *
   * @param request_header Unused request header (provided by rclcpp)
   * @param request Unused request object for Trigger
   * @param response Response object to set success and message fields
   */
  void add_5_sec_cb(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Log that the 'add 5 sec request' has been received
    {
      std::string msg = "Received add 5 seconds request for oven '" + oven_name_ + "'";
      RCLCPP_WARN_STREAM(this->get_logger(), msg);

      std_msgs::msg::String pub_msg;
      pub_msg.data = msg;
      publisher_->publish(pub_msg);
    }

    // Add time to the Oven
    // check the remaining time on the oven
    auto time_rem = end_time_ - std::chrono::steady_clock::now();
    int time_rem_sec = std::chrono::duration_cast<std::chrono::milliseconds>(time_rem).count();

    if (time_rem_sec > 0)  // if the oven is already running, add time to end_time_
    {
      end_time_ = end_time_ + std::chrono::duration(5s);

      std::string msg = "Added 5 seconds to the oven '" + oven_name_ + "' runtime";
      response->success = true;
      response->message = msg;

      // it does't need to be error, but just for the assignment
      RCLCPP_ERROR_STREAM(this->get_logger(), msg);
    }
    else  // if the oven is not already running, end_time_ = current_time = 5 sec
    {
      end_time_ = std::chrono::steady_clock::now() + std::chrono::duration(5s);

      std::string msg = "Starting the oven '" + oven_name_ + "', and running for 5 seconds";
      response->success = true;
      response->message = "Starting the oven, and running for 5 seconds";

      // it does't need to be fatal, but just for the assignment
      RCLCPP_FATAL_STREAM(this->get_logger(), msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;                        /**< Timer for periodic status publishing. */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; /**< Publisher for oven status messages. */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr add_5_sec_service_; /**< Service to add 5 seconds. */
  std::chrono::time_point<std::chrono::steady_clock> end_time_; /**< Time point when oven runtime ends. */
  std::string oven_name_; /**< Name identifier for this oven instance (from parameter "name"). */
};

/**
 * @brief Program entrypoint for the oven_publisher node.
 *
 * Initializes ROS, creates the node, spins it and shuts down cleanly.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OvenPublisher>());
  rclcpp::shutdown();
  return 0;
}
