#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("concurrent_trigger_node");

    // Create service clients for the left and right services
    auto left_client = node->create_client<Trigger>("left_start_sync");
    auto right_client = node->create_client<Trigger>("right_start_sync");

    // Wait for each service to be available
    while (!left_client->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Waiting for left_start_sync service...");
    }
    while (!right_client->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Waiting for right_start_sync service...");
    }

    // Create request messages (empty since Trigger has no fields)
    auto left_request = std::make_shared<Trigger::Request>();
    auto right_request = std::make_shared<Trigger::Request>();

    // Trigger both services concurrently by sending asynchronous requests
    auto left_future = left_client->async_send_request(left_request);
    auto right_future = right_client->async_send_request(right_request);

    // Enter a loop that periodically spins to process callbacks
    // and checks whether both service calls have completed.
    rclcpp::WallRate loop_rate(10ms);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (left_future.wait_for(0s) == std::future_status::ready &&
            right_future.wait_for(0s) == std::future_status::ready) {
            break;
        }
        loop_rate.sleep();
    }

    // Check and process the response from left_start_sync
    if (left_future.valid()) {
        auto left_response = left_future.get();
        RCLCPP_INFO(node->get_logger(), "Left service response: %s",
                    left_response->message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call left_start_sync service");
    }

    // Check and process the response from right_start_sync
    if (right_future.valid()) {
        auto right_response = right_future.get();
        RCLCPP_INFO(node->get_logger(), "Right service response: %s",
                    right_response->message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call right_start_sync service");
    }

    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}
