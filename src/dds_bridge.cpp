#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include "BatteryStatePubSubTypes.h"
#include "BatteryStateListener.h"
#include "TwistPubSubTypes.h"

#include <fastrtps/Domain.h>

using namespace std::chrono_literals;

using std::placeholders::_1;

class DDS_Bridge : public rclcpp::Node
{
public:
    DDS_Bridge()
        : Node("dds_bridge")
    {
        count_ = 0;
        batt_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/cyclonedds/battery", 1);
        // Create RTPSParticipant

        eprosima::fastrtps::ParticipantAttributes PParam;
        PParam.rtps.setName("fastrtps_participant_subscriber");
        mp_participant = eprosima::fastrtps::Domain::createParticipant(PParam);

        // //Register the type
        eprosima::fastrtps::Domain::registerType(mp_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&batteryStatePubSubType));
        eprosima::fastrtps::Domain::registerType(mp_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&twistPubSubType));

        // Create Subscriber
        batteryStateListener = new eprosima::BatteryStateListener(std::bind(&DDS_Bridge::battery_update, this, _1));
        twistPubListener = new eprosima::fastrtps::PublisherListener();

        eprosima::fastrtps::SubscriberAttributes Rparam;
        Rparam.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        Rparam.topic.topicDataType = batteryStatePubSubType.getName(); //Must be registered before the creation of the subscriber
        Rparam.topic.topicName = "rt/battery";
        mp_subscriber = eprosima::fastrtps::Domain::createSubscriber(mp_participant, Rparam, static_cast<eprosima::fastrtps::SubscriberListener *>(batteryStateListener));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cyclonedds/cmd_vel", 1, std::bind(&DDS_Bridge::cmd_vel_callback, this, _1));

        //CREATE THE PUBLISHER
        linearMsg.x(0);
        linearMsg.y(0);
        linearMsg.z(0);
        angularMsg.x(0);
        angularMsg.y(0);
        angularMsg.z(0);
        twistMsg.linear(linearMsg);
        twistMsg.angular(angularMsg);
        eprosima::fastrtps::PublisherAttributes Wparam;
        Wparam.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        Wparam.topic.topicDataType = twistPubSubType.getName();
        Wparam.topic.topicName = "rt/cmd_vel";
        mp_publisher = eprosima::fastrtps::Domain::createPublisher(mp_participant, Wparam, twistPubListener);
    }

private:
    void battery_update(float voltage)
    {
        auto message = sensor_msgs::msg::BatteryState();
        message.voltage = voltage;
        RCLCPP_INFO(this->get_logger(), "FastRTPS [/battery] ==> CycloneDDS [/cyclonedds/battery] message type [sensor_msgs::msg::BatteryState]");
        batt_pub_->publish(message);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "CycloneDDS [/cyclonedds/cmd_vel] ==> FastRTPS [/cmd_vel] message type [geometry_msgs::msg::Twist]");
        linearMsg.x(msg->linear.x);
        angularMsg.z(msg->angular.z);
        twistMsg.linear(linearMsg);
        twistMsg.angular(angularMsg);
        mp_publisher->write((void *)&twistMsg);
    }

    eprosima::fastrtps::Participant *mp_participant;
    eprosima::fastrtps::Subscriber *mp_subscriber;
    eprosima::fastrtps::Publisher *mp_publisher;
    eprosima::sensor_msgs::msg::BatteryStatePubSubType batteryStatePubSubType;
    eprosima::geometry_msgs::msg::TwistPubSubType twistPubSubType;
    eprosima::geometry_msgs::msg::Vector3 linearMsg;
    eprosima::geometry_msgs::msg::Vector3 angularMsg;
    eprosima::geometry_msgs::msg::Twist twistMsg;

    eprosima::BatteryStateListener *batteryStateListener;
    eprosima::fastrtps::PublisherListener *twistPubListener;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DDS_Bridge>());
    rclcpp::shutdown();
    return 0;
}