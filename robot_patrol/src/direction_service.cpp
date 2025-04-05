#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node
{
public:
  DirectionService() : Node("direction_service")
  {
    service_ = this->create_service<robot_patrol::srv::GetDirection>(
      "/direction_service", std::bind(&DirectionService::handle_request, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Direction Service Server Ready");
  }

private:
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;

  void handle_request(
    const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
    std::shared_ptr<robot_patrol::srv::GetDirection::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Service Requested");

    auto ranges = request->laser_data.ranges;
    size_t n = ranges.size();
    if (n < 3) {
      response->direction = "forward";
      return;
    }

    size_t sec = n / 3;
    double total_left = 0.0, total_front = 0.0, total_right = 0.0;

    for (size_t i = 0; i < sec; ++i)
      total_right += ranges[i];
    for (size_t i = sec; i < 2 * sec; ++i)
      total_front += ranges[i];
    for (size_t i = 2 * sec; i < n; ++i)
      total_left += ranges[i];

    if (total_front > 0.35 * sec)
      response->direction = "forward";
    else if (total_left > total_right)
      response->direction = "left";
    else
      response->direction = "right";

    RCLCPP_INFO(this->get_logger(), "Service Completed - Direction: %s", response->direction.c_str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
