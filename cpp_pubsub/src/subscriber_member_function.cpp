/**
 * @file subscriber_member_function.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
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
