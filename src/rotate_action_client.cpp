#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "midterm_ros2pkg/action/rotate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class ActionClient : public rclcpp::Node {
  public:
    using Rotate = midterm_ros2pkg::action::Rotate;
    using GoalHandleRotate = rclcpp_action::ClientGoalHandle<Rotate>;

    explicit ActionClient(double angle_deg, 
                          const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
        : Node("rotate_action_client", node_options),
          angle_rad(angle_deg * M_PI / 180.0),
          goal_done(false)
    {
      client_ptr_ = rclcpp_action::create_client<Rotate>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "rotate");

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ActionClient::send_goal, this));
    }

    bool is_goal_done() const {
      return goal_done;
    }

  private:
    rclcpp_action::Client<Rotate>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double angle_rad;
    bool goal_done;

    void send_goal() {
      timer_->cancel();
      goal_done = false;

      if (!client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        return;
      }

      if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        goal_done = true;
        return;
      }

      auto goal_msg = Rotate::Goal();
      goal_msg.angle = angle_rad;

      RCLCPP_INFO(this->get_logger(), "Sending goal: %.2f deg (%.2f rad)",
                angle_rad * 180.0 / M_PI, angle_rad);

      auto options = rclcpp_action::Client<Rotate>::SendGoalOptions();
      options.goal_response_callback =
        std::bind(&ActionClient::goal_response_callback, this, std::placeholders::_1);
      options.feedback_callback =
        std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      options.result_callback =
        std::bind(&ActionClient::result_callback, this, std::placeholders::_1);

      client_ptr_->async_send_goal(goal_msg, options);
    }

    void goal_response_callback(GoalHandleRotate::SharedPtr goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

    void feedback_callback(
      GoalHandleRotate::SharedPtr,
      const std::shared_ptr<const Rotate::Feedback> feedback) 
    {
      double remaining_rad = feedback->remaining_angle;
      double remaining_deg = remaining_rad * 180.0 / M_PI;
      RCLCPP_INFO(this->get_logger(), "Remaining angle: %.2f deg (%.2f rad)",
                        remaining_deg, remaining_rad);
    }

    void result_callback(const GoalHandleRotate::WrappedResult &result) {
      goal_done = true;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
    std::cout << "Usage: ros2 run cpp_action_client rotate_action_client <angle_deg>" << std::endl;
    return 1;
  }

  double angle_deg = std::stod(argv[1]);
  auto action_client = std::make_shared<ActionClient>(angle_deg);

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}
