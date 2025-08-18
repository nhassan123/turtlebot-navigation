#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class TurtlebotController : public rclcpp::Node
{
public:
    TurtlebotController() : Node("turtlebot3_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtlebotController::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Turtlebot3 Contoller Node has been started.");

    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.1;

        publisher_->publish(message);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotController>());
    rclcpp::shutdown();
    return 0;
}