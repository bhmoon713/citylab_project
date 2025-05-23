#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <cmath>  // for std::isfinite

using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node
{
public:
  DirectionService() : Node("direction_service_node")
  {
    service_ = this->create_service<robot_patrol::srv::GetDirection>(
      "/direction_service",
      std::bind(&DirectionService::handle_service, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Direction Service Server Ready");
  }

private:
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;

  void handle_service(
    const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
    std::shared_ptr<robot_patrol::srv::GetDirection::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Service Requested");

    const auto &ranges = request->laser_data.ranges;
    size_t n = ranges.size();
    // if (n < 3) {
    //   RCLCPP_WARN(this->get_logger(), "Not enough scan data");
    //   response->direction = "forward";
    //   return;
    // }


    // Define sector boundaries
    size_t right_start = n * 1 / 4;
    size_t right_end   = n * 5 / 12;
    size_t front_start = n * 5 / 12;
    size_t front_end   = n * 7 / 12;
    size_t left_start  = n * 7 / 12;
    size_t left_end    = n * 3 / 4;

    // Accumulate distances..&& i < n
    // double total_dist_sec_right = 0.0;
    // for (size_t i = right_start; i < right_end ; ++i)
    //   total_dist_sec_right += ranges[i];

    double total_dist_sec_right = 0.0;
    for (size_t i = right_start; i < right_end; ++i) {
    if (std::isfinite(ranges[i])) {  // excludes both inf and NaN
        total_dist_sec_right += ranges[i];
        }
    }

    // double total_dist_sec_front = 0.0;
    // for (size_t i = front_start; i < front_end ; ++i)
    //   total_dist_sec_front += ranges[i];

    double total_dist_sec_front = 0.0;
    for (size_t i = front_start; i < front_end; ++i) {
    if (std::isfinite(ranges[i])) {
        total_dist_sec_front += ranges[i];
        }
    }

    // double total_dist_sec_left = 0.0;
    // for (size_t i = left_start; i < left_end ; ++i)
    //   total_dist_sec_left += ranges[i];


    double total_dist_sec_left = 0.0;
    for (size_t i = left_start; i < left_end; ++i) {
    if (std::isfinite(ranges[i])) {
        total_dist_sec_left += ranges[i];
        }
    }

    float front_distance = ranges[n / 2];  // Center ray for obstacle detection

    auto min_it = std::min_element(ranges.begin() + 220, ranges.begin() + 440);
    float min_front = (min_it != ranges.end()) ? *min_it : std::numeric_limits<float>::infinity();


    RCLCPP_INFO(this->get_logger(),
      "front_dist: %.2f | Total (R: %.2f, F: %.2f, L: %.2f)",
      front_distance, total_dist_sec_right, total_dist_sec_front, total_dist_sec_left);

    // Decision logic
    if (front_distance < 0.35 || min_front < 0.3) {
    // if (total_dist_sec_front < 0.35 *110  || ranges[165] <0.3 || ranges[495] <0.3) {
    // Obstacle detected — evaluate sectors
    RCLCPP_INFO(this->get_logger(),
                "Obstacle detected | front_distance: %.2f | min_front: %.2f",
                front_distance, min_front);
        if (total_dist_sec_front >= total_dist_sec_left && total_dist_sec_front >= total_dist_sec_right) {
            response->direction = "forward";
        } else if (total_dist_sec_left >= total_dist_sec_front && total_dist_sec_left >= total_dist_sec_right) {
            response->direction = "left";
        } else {
            response->direction = "right";
        }

    RCLCPP_INFO(this->get_logger(), "Chosen direction: %s", response->direction.c_str());
    } else {
    response->direction = "forward";
    RCLCPP_INFO(this->get_logger(), "Path is clear — go forward");
    }
    RCLCPP_INFO(this->get_logger(), "Service Completed");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
