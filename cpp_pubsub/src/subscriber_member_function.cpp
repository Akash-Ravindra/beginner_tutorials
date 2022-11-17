/**
 * @file subscriber_member_function.cpp
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
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
/// @brief This class is used to subscribe to a topic and print the message
/// The class inherits from the rclcpp::Node class
/// The class has a subscription member function which subscribes to a topic
/// and prints the message
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("custom_node_subscriber") {
    // Create a subscription
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "custom_topic", 10,
        std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /// @brief This function is used to print the message received from the
  /// @param msg
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "The publisher said -> '%s'",
                msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/// @brief This function is used to create a subscriber node and spin it
/// @param argc
/// @param argv
/// @return
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
