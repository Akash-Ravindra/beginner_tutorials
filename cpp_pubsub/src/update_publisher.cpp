/**
 * @file update_publisher.cpp
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
#include <cstdlib>
#include <memory>

#include "custom_interfaces/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::string update_request = "";
  if (argc == 1) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                        "usage: add_two_ints_client X Y");
    return 1;
  } else {
    for (int i = 1; i < argc; i++) {
      update_request.append(argv[i]);
      update_request.append(" ");
    }
  }
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("change_string_client");
  rclcpp::Client<custom_interfaces::srv::ChangeString>::SharedPtr client =
      node->create_client<custom_interfaces::srv::ChangeString>(
          "change_string");

  auto request =
      std::make_shared<custom_interfaces::srv::ChangeString::Request>();
  request->input = update_request;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_FATAL_STREAM(
          rclcpp::get_logger("rclcpp"),
          "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       ("Result : " + std::to_string(result.get()->success)));
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                        "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
