#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("sub")
    {
        batt_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/cyclonedds/battery", 1, std::bind(&MinimalSubscriber::battery_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Battery status subscribed");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Battery status received");
        RCLCPP_INFO(this->get_logger(), "Voltage: '%f'", msg->voltage);
    }
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batt_sub_;
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}