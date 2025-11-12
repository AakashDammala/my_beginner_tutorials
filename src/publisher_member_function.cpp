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

class OvenPublisher : public rclcpp::Node
{
 public:
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

    RCLCPP_DEBUG_STREAM(this->get_logger(), "The oven has been powered up!");
  }

 private:
  void timer_callback()
  {
    auto time_rem = end_time_ - std::chrono::steady_clock::now();

    if (time_rem < std::chrono::steady_clock::duration::zero())
    {
      std::string msg = "The Oven is now ready to use now";
      RCLCPP_INFO_STREAM(this->get_logger(), msg);

      std_msgs::msg::String pub_msg;
      pub_msg.data = msg;
      publisher_->publish(pub_msg);
    }
    else
    {
      int time_rem_sec = std::chrono::duration_cast<std::chrono::seconds>(time_rem).count();

      std::string msg =
          "The Oven is running, time remaining: " + std::to_string(time_rem_sec) + " seconds";
      RCLCPP_INFO_STREAM(this->get_logger(), msg);

      std_msgs::msg::String pub_msg;
      pub_msg.data = msg;
      publisher_->publish(pub_msg);
    }
  }

  void add_5_sec_cb(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Log that the 'add 5 sec request' has been received
    {
      std::string msg = "Received add 5 seconds request";
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

      std::string msg = "Added 5 seconds to the oven runtime";
      response->success = true;
      response->message = msg;

      // it does't need to be error, but just for the assignment
      RCLCPP_ERROR_STREAM(this->get_logger(), msg);
    }
    else  // if the oven is not already running, end_time_ = current_time = 5 sec
    {
      end_time_ = std::chrono::steady_clock::now() + std::chrono::duration(5s);

      std::string msg = "Starting the oven, and running for 5 seconds";
      response->success = true;
      response->message = "Starting the oven, and running for 5 seconds";

      // it does't need to be fatal, but just for the assignment
      RCLCPP_FATAL_STREAM(this->get_logger(), msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr add_5_sec_service_;
  std::chrono::time_point<std::chrono::steady_clock> end_time_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OvenPublisher>());
  rclcpp::shutdown();
  return 0;
}
