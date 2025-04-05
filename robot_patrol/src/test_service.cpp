#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service") {
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&TestService::laserCallback, this, std::placeholders::_1));

    client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");

    RCLCPP_INFO(this->get_logger(), "Test Service Client Ready");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for service...");
      return;
    }

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = *msg;

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Service Response: %s",
                  future.get()->direction.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
  }
};

// ✅ Add this to fix the error
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestService>());
  rclcpp::shutdown();
  return 0;
}
