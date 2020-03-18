#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include "BatteryStatePubSubTypes.h"
#include "BatteryStateListener.h"

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

        // Create Subscriber
        batteryStateListener = new eprosima::BatteryStateListener(std::bind(&DDS_Bridge::battery_update, this, _1));

        eprosima::fastrtps::SubscriberAttributes Rparam;
        Rparam.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        Rparam.topic.topicDataType = batteryStatePubSubType.getName(); //Must be registered before the creation of the subscriber
        Rparam.topic.topicName = "rt/battery";
        mp_subscriber = eprosima::fastrtps::Domain::createSubscriber(mp_participant, Rparam, static_cast<eprosima::fastrtps::SubscriberListener *>(batteryStateListener));
    }

private:
    void battery_update(float voltage)
    {
        auto message = sensor_msgs::msg::BatteryState();
        message.voltage = voltage;
        RCLCPP_INFO(this->get_logger(), "FastRTPS [/battery] ==> CycloneDDS [/cyclonedds/battery] message type [sensor_msgs::msg::BatteryState]");
        batt_pub_->publish(message);
    }

    eprosima::fastrtps::Participant *mp_participant;
    eprosima::fastrtps::Subscriber *mp_subscriber;
    eprosima::sensor_msgs::msg::BatteryStatePubSubType batteryStatePubSubType;

    eprosima::BatteryStateListener *batteryStateListener;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
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