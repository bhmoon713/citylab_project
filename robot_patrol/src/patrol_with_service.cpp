#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

class RobotPatrolWithService : public rclcpp::Node {
public:
  RobotPatrolWithService() : Node("robot_patrol_service_node") {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&RobotPatrolWithService::scanCallback, this, std::placeholders::_1));

    client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotPatrolWithService::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "🚗 Robot Patrol with Service Node Started");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan latest_scan_;
  bool scan_received_ = false;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = *msg;
    scan_received_ = true;

    int scanN = msg->ranges.size();
    float left = msg->ranges[scanN * 3 / 4];
    float front = msg->ranges[scanN * 2 / 4];
    float right = msg->ranges[scanN * 1 / 4];

    RCLCPP_INFO(this->get_logger(),
                "📡 Laser size: %zu | Left %.2f Front %.2f Right: %.2f",
                msg->ranges.size(), left, front, right);
  }

  void timerCallback() {
    if (!scan_received_) return;

    auto cmd = geometry_msgs::msg::Twist();

    // Check front distance
    int scanN = latest_scan_.ranges.size();
    float front = latest_scan_.ranges[scanN * 2 / 4];

    if (front > 0.35) {
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "🟢 Path is clear — Going forward");
    } else {
      // Call direction_service
      if (!client_->wait_for_service(std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), "❗ Waiting for /direction_service...");
        return;
      }

      auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
      request->laser_data = latest_scan_;

      auto future = client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        std::string direction = future.get()->direction;
        RCLCPP_INFO(this->get_logger(), "➡️ Direction from service: %s", direction.c_str());

        if (direction == "forward") {
          cmd.linear.x = 0.1;
          cmd.angular.z = 0.0;
        } else if (direction == "left") {
          cmd.linear.x = 0.1;
          cmd.angular.z = 0.5;
        } else if (direction == "right") {
          cmd.linear.x = 0.1;
          cmd.angular.z = -0.5;
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "⚠️ Failed to call direction service");
      }
    }

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrolWithService>());
  rclcpp::shutdown();
  return 0;
}
