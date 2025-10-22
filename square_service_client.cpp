#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using Empty = std_srvs::srv::Empty;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto service_client = rclcpp::Node::make_shared("square_service_client");
    auto client_ = service_client->create_client<Empty>("square_service");

    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(service_client->get_logger(), "Client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(service_client->get_logger(), "Waiting for service to appear...");
    }

    auto request = std::make_shared<Empty::Request>();
    auto result_future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(service_client, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(service_client->get_logger(), "Service call failed :(");
        client_->remove_pending_request(result_future);
        return 1;
    }

    RCLCPP_INFO(service_client->get_logger(), "Service call succeeded.");
    rclcpp::shutdown();
    return 0;
}