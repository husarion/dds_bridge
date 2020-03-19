#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class TfBroadcasterTest : public rclcpp::Node
{
public:
    TfBroadcasterTest() : Node("tf_test")
    {
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(500ms, std::bind(&TfBroadcasterTest::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped current_transform;
        current_transform.header.frame_id = "main_frame";
        current_transform.header.stamp = this->now();
        current_transform.child_frame_id = "child_frame";
        current_transform.transform.translation.x = rand();
        current_transform.transform.translation.y = rand();
        current_transform.transform.translation.z = rand();
        current_transform.transform.rotation.x = rand();
        current_transform.transform.rotation.y = rand();
        current_transform.transform.rotation.z = rand();
        current_transform.transform.rotation.w = rand();
        tf_broadcaster->sendTransform(current_transform);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfBroadcasterTest>());
    rclcpp::shutdown();
    return 0;
}
