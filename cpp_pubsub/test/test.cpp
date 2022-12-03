/**
 * @file test.cpp
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
#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

#include "custom_interfaces/srv/change_string.hpp"
#include "publisher_node.hpp"

using namespace std::chrono_literals;

void executor_thread(rclcpp::executors::SingleThreadedExecutor* executor) {
  // spin the executor
  while (rclcpp::ok()) {
    executor->spin_once();
    // sleep for 10ms
    rclcpp::sleep_for(100ms);
  }
}

namespace minimal_integration_test {
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }
  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    test_node_ = std::make_shared<MinimalPublisher>();
    // Add the node to the executor
    executor_.add_node(test_node_);
    // Start the executor thread
    executor_thread_ = std::thread(executor_thread, &executor_);
    executor_thread_.detach();
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH TEARDOWN");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
};

TEST_F(TaskPlanningFixture, TestTopicMsg) {
  std::string test_string = "Hello World";
  // Create a client to send a request to the service
  rclcpp::Client<custom_interfaces::srv::ChangeString>::SharedPtr client =
      node_->create_client<custom_interfaces::srv::ChangeString>(
          "change_string");  // Send a request to the service
  auto request =
      std::make_shared<custom_interfaces::srv::ChangeString::Request>();
  request->input = test_string;
  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }
  // Send the request
  auto result = client->async_send_request(request);

  std::string sub_msg;
  // Create a subscriber to receive the message
  auto sub = node_->create_subscription<std_msgs::msg::String>(
      "custom_topic", 10, [&sub_msg](const std_msgs::msg::String& msg) {
        sub_msg = msg.data;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard:  '%s'",
                    sub_msg.c_str());
      });
  // Wait for the message to be received
  while (sub_msg.empty()) {
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(100ms);
  }
  // Check if the message received is the same as the one set in the request
  ASSERT_EQ(sub_msg.find(test_string) != std::string::npos, true);
}

}  // namespace minimal_integration_test

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}