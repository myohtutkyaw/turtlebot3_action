#include <memory>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "midterm_ros2pkg/action/rotate.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using Rotate = midterm_ros2pkg::action::Rotate;
using GoalHandleRotate = rclcpp_action::ServerGoalHandle<Rotate>;

class ActionServer : public rclcpp::Node {
public:
  ActionServer() : Node("rotate_action_server") {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<Rotate>(
      this,
      "rotate",
      std::bind(&ActionServer::handle_goal, this, _1, _2),
      std::bind(&ActionServer::handle_cancel, this, _1),
      std::bind(&ActionServer::handle_accepted, this, _1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&ActionServer::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<Rotate>::SharedPtr server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double yaw_{0.0};

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto& q = msg->pose.pose.orientation;
    yaw_ = std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)),
                      1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))));
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const Rotate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: Rotate %.2f deg (%.2f rad)",
                goal->angle * 180.0 / M_PI, goal->angle);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRotate>) {
    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRotate> goal_handle) {
    std::thread{std::bind(&ActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleRotate> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Rotate::Feedback>();
    auto result = std::make_shared<Rotate::Result>();
    rclcpp::Rate rate(10);
    const double Kp = 1.5;
    const double tolerance = 0.5 * M_PI / 180.0;
    const double target_yaw = normalize_angle(goal->angle);

    while (rclcpp::ok()) {
      const double yaw_error = normalize_angle(target_yaw - yaw_);
      feedback->remaining_angle = yaw_error;
      goal_handle->publish_feedback(feedback);
      if (std::abs(yaw_error) < tolerance) break;
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = Kp * yaw_error;
      vel_pub_->publish(cmd);
      rate.sleep();
    }

    stop_robot();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Rotation to angle complete.");
  }

  void stop_robot() {
    geometry_msgs::msg::Twist stop_cmd;
    vel_pub_->publish(stop_cmd);
  }

  static double normalize_angle(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) angle += 2.0 * M_PI;
    return angle - M_PI;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


