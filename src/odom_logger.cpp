#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

double yaw_from_quat(double x, double y, double z, double w) {
  const double siny_cosp = 2.0 * (w*z + x*y);
  const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class OdomLogger : public rclcpp::Node {
public:
  OdomLogger() : Node("odom_logger"), count_(0) {
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg){
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        const double yaw = yaw_from_quat(q.x, q.y, q.z, q.w);
        if ((count_++ % 10) == 0) {  // print every 10th message
          RCLCPP_INFO(this->get_logger(), "x=%.3f  y=%.3f  yaw=%.1f°",
                      p.x, p.y, yaw * 180.0/M_PI);
        }
      });
    RCLCPP_INFO(this->get_logger(), "odom_logger started, listening to /odom");
  }
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  size_t count_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomLogger>());
  rclcpp::shutdown();
  return 0;
}
