#ifndef TURTLEBOT_CONTROLLER_HPP
#define TURTLEBOT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class TurtlebotController : public rclcpp::Node
{
public:
    TurtlebotController();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_scan_sub;
    rclcpp::TimerBase::SharedPtr _timer;

    bool _obstacle_detected;

};

#endif
