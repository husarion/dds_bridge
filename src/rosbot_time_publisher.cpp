#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class ROSbotTimePublisher : public rclcpp::Node
{
public:
    ROSbotTimePublisher() : Node("time_publisher")
    {
        publisher_ = this->create_publisher<builtin_interfaces::msg::Time>("/rosbot_time", 1);
        timer_ = this->create_wall_timer(20ms, std::bind(&ROSbotTimePublisher::timer_callback, this));
        current_time = new builtin_interfaces::msg::Time();
        RCLCPP_INFO(this->get_logger(), "Clock publisher initilized");
    }

private:
    void timer_callback()
    {
        *current_time = this->now();
        publisher_->publish(current_time);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
    builtin_interfaces::msg::Time *current_time;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROSbotTimePublisher>());
    rclcpp::shutdown();
    return 0;
}