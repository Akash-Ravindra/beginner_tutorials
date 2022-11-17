/**
 * @file publisher_member_function.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-16
 * 
 * MIT License
 * Copyright (c) 2022 Akash Ravindra
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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

/// @brief This class is used to publish a string message to a topic
/// The class inherits from the rclcpp::Node class
/// The class has a publisher member function which publishes a string message
/// to a topic
/// The class has a timer member function which calls the publisher member
/// function at a specified frequency set using parameter server
/// The class has a service member function which updates the string message
/// to be published
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()
      : Node("Custom_Node_Publisher"), msg_("The power-ball number is = ") {
    // Declare and acquire parameters from the parameter server
    this->declare_parameter("talker_f", 1.0);
    auto frequency =
        this->get_parameter("talker_f").get_parameter_value().get<double>();
    // Create a publisher
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("custom_topic", 10);
    // Create a service server
    service_ = this->create_service<custom_interfaces::srv::ChangeString>(
        "change_string",
        std::bind(&MinimalPublisher::change_string_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    if (msg_ == "The power-ball number is = ") {
      RCLCPP_WARN_STREAM(this->get_logger(), "Using Default String");
    }
    // Create a timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1 / frequency),
        std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  /// @brief This function is used to publish a string message to a topic
  void timer_callback() {
    auto message = std_msgs::msg::String();
    unsigned int seed =
        std::chrono::system_clock::now().time_since_epoch().count();
    int random_number = rand_r(&seed) % 100;
    message.data = msg_ + std::to_string(random_number);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
    publisher_->publish(message);
  }
  /// @brief This function is used to update the string message to be published
  /// @param request
  /// @param response
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
/// @brief  Main function for the publisher node
/// @param argc
/// @param argv
/// @return
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
