#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm> // For min/max element
#include <tuple>

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("robot_patrol_node") {
    // === Publishers and Subscribers ===
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&RobotPatrol::scanCallback, this, std::placeholders::_1));

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), // 10Hz
                                std::bind(&RobotPatrol::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Robot Patrol Node Started");
  }

private:
  // === Member Variables ===
  float min_value_;     // Distance to closest obstacle
  float min_direction_; // Angle to closest obstacle
  float max_value_;     // Max distance between 164~494
  float max_direction_; // Angle to max distance
  float left_= 0.0;
  float front_= 0.0;
  float right_ = 0.0;

  // === Callbacks ===
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Simulation total laser value 660
    // Real robot total laser value 720
    int scanN = msg->ranges.size();
    left_ = msg->ranges[scanN * 3 / 4];
    front_ = msg->ranges[scanN * 2 / 4];
    right_ = msg->ranges[scanN * 1 / 4];

    RCLCPP_INFO(this->get_logger(),
                "📡 Laser size: %zu | Left %.2f Front %.2f Right: %.2f ",
                msg->ranges.size(), left_, front_, right_);

    // --- Min value in full range ---
    // auto min_it = std::min_element(msg->ranges.begin() + 164,
    // msg->ranges.begin() + 494); min_value_ = *min_it; int min_index =
    // std::distance(msg->ranges.begin(), min_it); min_direction_ = (min_index -
    // 329) * 6.28 / 660;

    auto min_it = std::min_element(msg->ranges.begin() + scanN / 4,
                                   msg->ranges.begin() + scanN * 3 / 4);
    min_value_ = *min_it;
    int min_index = std::distance(msg->ranges.begin(), min_it);
    min_direction_ = (min_index - scanN / 2) * 6.28 / scanN;

    // --- Max value in specific range 164~494 ---
    // auto max_it = std::max_element(msg->ranges.begin() + 164,
    // msg->ranges.begin() + 494); max_value_ = *max_it; int max_index =
    // std::distance(msg->ranges.begin(), max_it); max_direction_ = (max_index -
    // 329) * 6.28 / 660;

    auto max_it = std::max_element(msg->ranges.begin() + scanN / 4,
                                   msg->ranges.begin() + scanN * 3 / 4);
    max_value_ = *max_it;
    int max_index = std::distance(msg->ranges.begin(), max_it);
    max_direction_ = (max_index - scanN / 2) * 6.28 / scanN;

    // --- Print for debug ---
    // RCLCPP_INFO(this->get_logger(), "📡 Laser size: %ld | Min: %.2f at %.2f
    // rad | Max: %.2f at %.2f rad",
    //             msg->ranges.size(), min_value_, min_direction_, max_value_,
    //             max_direction_);
  }

    void checkvicinity(geometry_msgs::msg::Twist &cmd) {
        if (min_value_ < 0.25 && min_direction_ < 0) {
            cmd.linear.x = 0.05;
            cmd.angular.z = 0.5;
            RCLCPP_INFO(this->get_logger(), "There is something on right");
        } else if (min_value_ < 0.25 && min_direction_ > 0) {
            cmd.linear.x = 0.05;
            cmd.angular.z = -0.5;
            RCLCPP_INFO(this->get_logger(), "There is something on left");
        }

    }

  void timerCallback() {
    auto cmd = geometry_msgs::msg::Twist();

    // === Obstacle Avoidance Logic ===
    if (front_ > 0.35) {
        cmd.linear.x = 0.1;
        cmd.angular.z = 0.0;
        checkvicinity(cmd);
        RCLCPP_INFO(this->get_logger(), "I am going forward");
    } else {
        cmd.linear.x = 0.1;
        cmd.angular.z = max_direction_ / 2;
        checkvicinity(cmd);
        RCLCPP_INFO(this->get_logger(), "I am turning to longest distance direction");
    }
    cmd_pub_->publish(cmd);
  }

  // === Publishers and Subscribers ===
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrol>());
  rclcpp::shutdown();
  return 0;
}
