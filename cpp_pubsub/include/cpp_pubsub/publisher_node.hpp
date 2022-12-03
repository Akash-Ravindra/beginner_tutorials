/**
 * @file publisher_node.hpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-03
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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
#ifndef CPP_PUBSUB_INCLUDE_CPP_PUBSUB_PUBLISHER_NODE_HPP_
#define CPP_PUBSUB_INCLUDE_CPP_PUBSUB_PUBLISHER_NODE_HPP_
#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "custom_interfaces/srv/change_string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

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
  MinimalPublisher();

 private:
  void broadcast_tf();
  /// @brief Timer callback function that publishes a string message to a topic
  /// and broadcasts a TF
  void timer_callback();
  /// @brief This function is used to update the string message to be published
  /// @param request
  /// @param response
  void change_string_callback(
      const std::shared_ptr<custom_interfaces::srv::ChangeString::Request>
          request,
      std::shared_ptr<custom_interfaces::srv::ChangeString::Response> response);
  geometry_msgs::msg::TransformStamped update_step_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<custom_interfaces::srv::ChangeString>::SharedPtr service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::string msg_;
};

#endif  // CPP_PUBSUB_INCLUDE_CPP_PUBSUB_PUBLISHER_NODE_HPP_
