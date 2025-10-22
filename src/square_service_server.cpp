#include <cmath>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class ServiceServer : public rclcpp::Node
{
public:
    ServiceServer() : Node("square_service_server"),
                      state(STATE_IDLE),
                      n_side(0),
                      side_length(0.5),
                      linear_speed(0.17),
                      angular_speed(0.7)
    {
        service_ = this->create_service<std_srvs::srv::Empty>(
            "square_service",
            std::bind(&ServiceServer::handle_service, this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3));

        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
            std::bind(&ServiceServer::odom_callback, this, std::placeholders::_1));
    }

private:
    const int STATE_IDLE = 0;
    const int STATE_MOVING_FORWARD = 1;
    const int STATE_WAIT_BEFORE_ROTATE = 2;
    const int STATE_ROTATING = 3;
    const int STATE_WAIT_AFTER_ROTATE = 4;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg = msg;

        auto q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        m.getRPY(roll, pitch, yaw);
    }

    void control_loop() {
        if (!odom_msg) return;

        geometry_msgs::msg::Twist cmd;

        double target_x = odom_msg->pose.pose.position.x;
        double target_y = odom_msg->pose.pose.position.y;

        double dx = target_x - current_x;
        double dy = target_y - current_y;
        double distance = std::sqrt((dx * dx) + (dy * dy));

        if (state == STATE_MOVING_FORWARD) {
            if (distance < side_length) {
                cmd.linear.x = linear_speed;
            } else {
                cmd.linear.x = 0.0;
                vel_pub->publish(cmd);

                state = STATE_WAIT_BEFORE_ROTATE;
                delay_timer_ = this->create_wall_timer(250ms, std::bind(&ServiceServer::start_rotating, this));
                return;
            }
        } else if (state == STATE_ROTATING) {
            double target_yaw = current_yaw + M_PI_2;
            double error = normalize_angle(target_yaw - yaw);

            if (std::abs(error) > 0.08) {
                cmd.angular.z = angular_speed;
            } else {
                cmd.angular.z = 0.0;
                vel_pub->publish(cmd);

                n_side++;
                if (n_side >= 4) {
                    stop_robot();
                    timer_->cancel();
                    state = STATE_IDLE;
                    return;
                }

                state = STATE_WAIT_AFTER_ROTATE;
                delay_timer_ = this->create_wall_timer(250ms, std::bind(&ServiceServer::start_moving_forward, this));
                return;
            }
        } else if (state == STATE_WAIT_BEFORE_ROTATE || state == STATE_WAIT_AFTER_ROTATE || state == STATE_IDLE) {
            return;
        }

        vel_pub->publish(cmd);
    }

    void handle_service(
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        const std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        if (!odom_msg) {
            RCLCPP_WARN(this->get_logger(), "No odometry received yet.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Service called: Moving robot in a square.");

        n_side = 0;
        state = STATE_MOVING_FORWARD;

        current_x = odom_msg->pose.pose.position.x;
        current_y = odom_msg->pose.pose.position.y;

        timer_ = this->create_wall_timer(50ms, std::bind(&ServiceServer::control_loop, this));
    }

    void start_moving_forward() {
        if (delay_timer_) delay_timer_->cancel();

        current_x = odom_msg->pose.pose.position.x;
        current_y = odom_msg->pose.pose.position.y;
        state = STATE_MOVING_FORWARD;
    }

    void start_rotating() {
        if (delay_timer_) delay_timer_->cancel();

        current_yaw = yaw;
        state = STATE_ROTATING;
    }

    void stop_robot() {
        geometry_msgs::msg::Twist stop_cmd;
        vel_pub->publish(stop_cmd);
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    int state;
    int n_side;
    double side_length;
    double linear_speed;
    double angular_speed;

    double current_x;
    double current_y;
    double current_yaw;

    double roll, pitch, yaw;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr delay_timer_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry::SharedPtr odom_msg;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto service_server = std::make_shared<ServiceServer>();
    rclcpp::spin(service_server);
    rclcpp::shutdown();
    return 0;
}