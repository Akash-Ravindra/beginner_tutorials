// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/srv/change_string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()
      : Node("Custom_Node_Publisher"), msg_("The power-ball number is = ") {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("custom_topic", 10);
    service_ = this->create_service<custom_interfaces::srv::ChangeString>("change_string", std::bind(&MinimalPublisher::change_string_callback, this, std::placeholders::_1, std::placeholders::_2));
    if(msg_ == "The power-ball number is = "){
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Using Default String");
    }
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    int random_number = rand() % 100;
    message.data = msg_ + std::to_string(random_number);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  void change_string_callback(const std::shared_ptr<custom_interfaces::srv::ChangeString::Request> request,
                        std::shared_ptr<custom_interfaces::srv::ChangeString::Response> response)
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Service Request Received");
    msg_ = request->input;
    response->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), ("Changing the string to: "+msg_));
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<custom_interfaces::srv::ChangeString>::SharedPtr service_;
  std::string msg_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
