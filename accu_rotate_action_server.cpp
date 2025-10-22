#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "midterm_ros2pkg/action/rotate.hpp"

using Rotate = midterm_ros2pkg::action::Rotate;
using GoalHandleRotate = rclcpp_action::ServerGoalHandle<Rotate>;

class ActionServer : public rclcpp::Node {
    public:
        ActionServer() : Node("rotate_action_server") {
            using namespace std::placeholders;

            server_ = rclcpp_action::create_server<Rotate>(this, "rotate",
                std::bind(&ActionServer::handle_goal, this, _1, _2),
                std::bind(&ActionServer::handle_cancel, this, _1),
                std::bind(&ActionServer::handle_accepted, this, _1));

            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
                                std::bind(&ActionServer::odom_callback, this, _1));
    }

    private:
        rclcpp_action::Server<Rotate>::SharedPtr server_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

        double yaw;

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            auto q = msg->pose.pose.orientation;
            yaw = std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)), 
                                1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))));
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Rotate::Goal> goal) {
            RCLCPP_INFO(this->get_logger(), "Received goal: Rotate %.2f deg (%.2f rad)", goal->angle * 180 / M_PI, goal->angle);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRotate>) {
            RCLCPP_INFO(this->get_logger(), "Goal canceled.");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleRotate> goal_handle) {
           std::thread{std::bind(&ActionServer::execute, this, goal_handle)}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleRotate> goal_handle) {
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Rotate::Feedback>();
            auto result = std::make_shared<Rotate::Result>();

            rclcpp::Rate rate(10); // 10 Hz

            double Kp = 1.5;
            double tolerance = 0.5 * M_PI / 180.0;
            double previous_yaw = yaw;
            double accumulated_angle = 0.0;

            RCLCPP_INFO(this->get_logger(), "Starting accumulated rotation: %.2f rad", goal->angle);

            while (rclcpp::ok()) {
                double delta_yaw = normalize_angle(yaw - previous_yaw);
                accumulated_angle += delta_yaw;
                previous_yaw = yaw;

                double yaw_error = goal->angle - accumulated_angle;
                feedback->remaining_angle = yaw_error;
                goal_handle->publish_feedback(feedback);

                if (std::abs(yaw_error) < tolerance) {
                    // std::cout << "Rotate is done" << std::endl;
                    break;
                }

                geometry_msgs::msg::Twist cmd;
                cmd.angular.z = Kp * yaw_error;
                // std::cout << "ang_vel: " << cmd.angular.z << std::endl;
                vel_pub->publish(cmd);
                
                rate.sleep();
            }

            stop_robot();
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Accumulated rotation complete.");
        }

        void stop_robot() {
            geometry_msgs::msg::Twist stop_cmd;
            vel_pub->publish(stop_cmd);
            // std::cout << "Stop robot" << std::endl;
        }

        double normalize_angle(double angle) {
            angle = std::fmod(angle + M_PI, 2 * M_PI);
            if (angle < 0) angle += 2 * M_PI;
            return angle - M_PI;
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}