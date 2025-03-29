#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <tuple>

class RobotPatrol : public rclcpp::Node
{
public:
  RobotPatrol() : Node("robot_patrol_node")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&RobotPatrol::scanCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10Hz
      std::bind(&RobotPatrol::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "robot_patrol_node started with separate callbacks!");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // ✅ Get max value & index between range 164~494
    auto max_it = std::max_element(msg->ranges.begin() + 164, msg->ranges.begin() + 494);
    float max_value = *max_it;
    int max_index = std::distance(msg->ranges.begin(), max_it);

    // ✅ Get min value & index (optional, for avoidance)
    auto min_it = std::min_element(msg->ranges.begin() + 164, msg->ranges.begin() + 494);
    min_value_ = *min_it;
    int min_index = std::distance(msg->ranges.begin(), min_it);
    min_direction_ = (min_index - 329) * 6.28 / 660;

    direction_ = (max_index - 329) * 6.28 / 660;

    // ✅ Optional: Print for debug
    //RCLCPP_INFO(this->get_logger(), "Laser Size: %ld", msg->ranges.size());
    RCLCPP_INFO(this->get_logger(), "Max Value: %.2f at index %d", max_value, max_index);
    RCLCPP_INFO(this->get_logger(), "Min Value: %.2f at index %d", min_value_, min_index);
  }

  void timerCallback()
  {
    geometry_msgs::msg::Twist cmd;

    if (min_value_ < 0.15) {
      cmd.linear.x = 0.0;
      if (min_direction_ < 0) {
        cmd.angular.z = 0.5;
      } else {
        cmd.angular.z = -0.5;
      }
    } else {
      cmd.linear.x = 0.1;
      cmd.angular.z = direction_ / 2;
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ✅ Stored variables
  float direction_ = 0.0;
  float min_value_ = 10.0;
  float min_direction_ = 0.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrol>());
  rclcpp::shutdown();
  return 0;
}
