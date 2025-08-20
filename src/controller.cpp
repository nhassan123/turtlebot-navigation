#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class TurtlebotController : public rclcpp::Node
{
public:
    TurtlebotController() : Node("turtlebot3_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscribe to laser scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&TurtlebotController::scan_callback, this, _1));


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtlebotController::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Turtlebot3 Contoller Node has been started.");

    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int num_ranges = msg->ranges.size();
        int mid_index = num_ranges / 2;

        // Check 45 degree range around the front
        double angle_range = 45.0 * M_PI / 180.0; // 15 degrees in radians
        int samples_per_angle = static_cast<int>(angle_range / msg->angle_increment);

        int start_index = std::max(0, mid_index - samples_per_angle);
        int end_index = std::min(mid_index + samples_per_angle, num_ranges - 1);

        obstacle_detected = false;
        for (int i = start_index; i <= end_index; i++) {
            if (msg->ranges[i] <= 0.2) {
                obstacle_detected = true;
                break;
            }
        }
    }

    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        if (obstacle_detected) {
            message.linear.x = 0.0;
            RCLCPP_WARN(this->get_logger(), "Obstacle detected ahead!");
        }
        else {
            message.linear.x = 0.1;
        }

        publisher_->publish(message);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool obstacle_detected;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotController>());
    rclcpp::shutdown();
    return 0;
}
