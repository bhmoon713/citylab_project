#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>  // for std::min_element
#include <tuple>      // for std::tuple

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
    // float right = msg->ranges[msg->ranges.size()* 1 / 4];
    // float front_right = msg->ranges[msg->ranges.size()* 1.5 / 4];
    // float front = msg->ranges[msg->ranges.size() / 2];
    // float front_left = msg->ranges[msg->ranges.size() * 2.5 / 4];
    // float left = msg->ranges[msg->ranges.size() * 3 / 4]; 
    
    // float right = msg->ranges[164];
    // float front = msg->ranges[329];
    // float left = msg->ranges[494];

//to get max value
    auto max_it = std::max_element(msg->ranges.begin() + 164, msg->ranges.begin() + 494);
    float max_value = *max_it;
    int max_index = std::distance(msg->ranges.begin(), max_it);

    RCLCPP_INFO(this->get_logger(), "Max value between 164~494: %.2f at index %d", max_value, max_index);
    RCLCPP_INFO(this->get_logger(), "Angle to max distance: %.2f at index %d", (max_index-329)*6.28/660, max_index);


    //RCLCPP_INFO(this->get_logger(), "Laser Size: %ld", msg->ranges.size());
    //RCLCPP_INFO(this->get_logger(), "Laser Readings → Front: %.2f m | Left: %.2f m | Right: %.2f m", front, left, right);
    auto cmd = geometry_msgs::msg::Twist();


    cmd.linear.x = 0.1;
    cmd.angular.z = (max_index-329)*6.28/660;
  
    //cmd_pub_->publish(cmd);
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