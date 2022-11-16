/**
 * @file publisher_member_function.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "custom_interfaces/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()
      : Node("Custom_Node_Publisher"), msg_("The power-ball number is = ") {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("custom_topic", 10);
    service_ = this->create_service<custom_interfaces::srv::ChangeString>(
        "change_string",
        std::bind(&MinimalPublisher::change_string_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    if (msg_ == "The power-ball number is = ") {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Using Default String");
    }
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    int random_number = rand_r(&seed) % 100;
    message.data = msg_ + std::to_string(random_number);
    RCLCPP_INFO(this->get_logger(), "Publishing: " + message.data);
    publisher_->publish(message);
  }
  void change_string_callback(
      const std::shared_ptr<custom_interfaces::srv::ChangeString::Request>
          request,
      std::shared_ptr<custom_interfaces::srv::ChangeString::Response>
          response) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Service Request Received");
    msg_ = request->input;
    response->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), ("Changing the string to: " + msg_));
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<custom_interfaces::srv::ChangeString>::SharedPtr service_;
  std::string msg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
