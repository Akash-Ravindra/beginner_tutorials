/**
 * @file publisher_node.cpp
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
#include "publisher_node.hpp"

MinimalPublisher::MinimalPublisher()
    : Node("Custom_Node_Publisher"), msg_("The power-ball number is = ") {
  // Declare and acquire parameters from the parameter server
  this->declare_parameter("talker_f", 1.0);
  auto frequency =
      this->get_parameter("talker_f").get_parameter_value().get<double>();

  // Create TF broadcaster
  tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  update_step_.header.frame_id = "world";
  update_step_.child_frame_id = "talk";
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

void MinimalPublisher::broadcast_tf() {
  // Create a transform
  update_step_.header.stamp = this->now();
  update_step_.transform.translation.x =
      update_step_.transform.translation.x + 1;
  update_step_.transform.translation.y =
      update_step_.transform.translation.y - 1;
  update_step_.transform.translation.z =
      update_step_.transform.translation.z + 5;
  tf2::Quaternion q;
  q.setRPY(0.785398, 0, 0.785398);
  update_step_.transform.rotation.x = q.x();
  update_step_.transform.rotation.y = q.y();
  update_step_.transform.rotation.z = q.z();
  update_step_.transform.rotation.w = q.w();
  // Broadcast the transform
  tfb_->sendTransform(update_step_);
}

void MinimalPublisher::timer_callback() {
  // Broadcast the transform
  broadcast_tf();
  // Create a string message
  auto message = std_msgs::msg::String();
  unsigned int seed =
      std::chrono::system_clock::now().time_since_epoch().count();
  int random_number = rand_r(&seed) % 100;
  message.data = msg_ + std::to_string(random_number);
  RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
  publisher_->publish(message);
}

void MinimalPublisher::change_string_callback(
    const std::shared_ptr<custom_interfaces::srv::ChangeString::Request>
        request,
    std::shared_ptr<custom_interfaces::srv::ChangeString::Response> response) {
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Service Request Received");
  msg_ = request->input;
  response->success = true;
  RCLCPP_INFO_STREAM(this->get_logger(), ("Changing the string to: " + msg_));
}
