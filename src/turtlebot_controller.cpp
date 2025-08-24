#include "turtlebot_navigation/turtlebot_controller.hpp"

using std::placeholders::_1;
const double ANGLE_HEADING_RANGE = 15.0;
const double DISTANCE_THRESHOLD = 0.5;

TurtlebotController::TurtlebotController() : Node("turtlebot3_controller"), _obstacle_detected(false)
{
    _vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    _laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TurtlebotController::scan_callback, this, _1));

    _timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TurtlebotController::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 Contoller Node has been started.");
}

void TurtlebotController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int scanner_range_size = msg->ranges.size();

    double heading_range_radians = ANGLE_HEADING_RANGE * M_PI / 180.0;
    int scan_section_size = static_cast<int>(heading_range_radians / msg->angle_increment);

    _obstacle_detected = false;
    // scan on the right set of heading direction
    for (int i = 0; i <= scan_section_size; i++) {
        if (msg->ranges[i] <= DISTANCE_THRESHOLD) {
            _obstacle_detected = true;
            return;
        }
    }
    
    // scan on the left side of heading direction
    for (int i = scanner_range_size - scan_section_size; i < scanner_range_size; i++) {
        if (msg->ranges[i] <= DISTANCE_THRESHOLD) {
            _obstacle_detected = true;
            return;
        }
    }
}

void TurtlebotController::timer_callback()
{
    auto message = geometry_msgs::msg::Twist();
    if (_obstacle_detected) {
        message.linear.x = 0.0;
        RCLCPP_WARN(this->get_logger(), "Obstacle detected ahead!");
    }
    else {
        message.linear.x = 0.1;
    }

    _vel_pub->publish(message);
}
