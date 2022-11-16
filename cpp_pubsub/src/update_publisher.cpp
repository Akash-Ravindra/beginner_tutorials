#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/change_string.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
    std::string update_request = "";
  if (argc == 1) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }
  else{
    for(int i=1; i<argc; i++){update_request.append(argv[i]);update_request.append(" ");}
  }
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("change_string_client");
  rclcpp::Client<custom_interfaces::srv::ChangeString>::SharedPtr client =
    node->create_client<custom_interfaces::srv::ChangeString>("change_string");

  auto request = std::make_shared<custom_interfaces::srv::ChangeString::Request>();
  request->input = update_request;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), ("Result : " + std::to_string(result.get()->success)));
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}