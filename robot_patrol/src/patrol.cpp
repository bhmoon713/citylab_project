#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotPatrol : public rclcpp::Node
{
public:
  RobotPatrol() : Node("robot_patrol_node")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&RobotPatrol::scanCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "robot_patrol_node started!");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // float front = msg->ranges[msg->ranges.size() / 2];
    // float left = msg->ranges[msg->ranges.size() * 1 / 4];
    // float right = msg->ranges[msg->ranges.size()* 3 / 4];
    float left = msg->ranges[0];
    float front_left = msg->ranges[180];
    float front = msg->ranges[360];
    float front_right = msg->ranges[540];
    float right = msg->ranges[719];
 RCLCPP_INFO(this->get_logger(), "Laser Readings → Front: %.2f m | Left: %.2f m | Right: %.2f m", front, left, right);
    auto cmd = geometry_msgs::msg::Twist();

  if (front > 1.0 && front_left > 1.0 && front_right > 1.0) {
    // ✅ No obstacle: Move forward
    cmd.linear.x = 0.5;
    cmd.angular.z = 0.0;
  } 
  else if (front < 1.0 || right < 1.0 || front_right < 1.0 ) {
    // ✅ Obstacle in front OR right: Turn left
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.5;
  }
  else if (left < 1.0 ) {
    // ✅ Obstacle in left: Turn right
    cmd.linear.x = 0.0;
    cmd.angular.z = -0.5;
  }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrol>());
  rclcpp::shutdown();
  return 0;
}