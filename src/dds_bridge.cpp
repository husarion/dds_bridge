#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
#include "PoseStampedPubSubTypes.h"
#include "PoseStampedListener.h"

#include <fastrtps/Domain.h>

using namespace std::chrono_literals;

using std::placeholders::_1;

class DDS_Bridge : public rclcpp::Node
{
public:
    DDS_Bridge()
        : Node("dds_bridge")
    {
        batt_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/cyclonedds/battery", 1);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cyclonedds/odom", 1);

        // Create RTPSParticipant
        eprosima::fastrtps::ParticipantAttributes PParam;
        PParam.rtps.setName("fastrtps_participant_subscriber");
        fastrtps_participant = eprosima::fastrtps::Domain::createParticipant(PParam);

        // //Register the type
        eprosima::fastrtps::Domain::registerType(fastrtps_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&batteryStatePubSubType));
        eprosima::fastrtps::Domain::registerType(fastrtps_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&poseStampedPubSubType));
        eprosima::fastrtps::Domain::registerType(fastrtps_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&twistPubSubType));

        // Create Subscriber
        batteryStateListener = new eprosima::BatteryStateListener(std::bind(&DDS_Bridge::battery_update, this, _1));
        poseStampedListener = new eprosima::PoseStampedListener(std::bind(&DDS_Bridge::pose_update, this, _1));
        twistPubListener = new eprosima::fastrtps::PublisherListener();

        eprosima::fastrtps::SubscriberAttributes Rparam;
        Rparam.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        Rparam.topic.topicDataType = batteryStatePubSubType.getName(); //Must be registered before the creation of the subscriber
        Rparam.topic.topicName = "rt/battery";
        battery_subscriber = eprosima::fastrtps::Domain::createSubscriber(fastrtps_participant, Rparam, static_cast<eprosima::fastrtps::SubscriberListener *>(batteryStateListener));

        eprosima::fastrtps::SubscriberAttributes poseSubAtt;
        poseSubAtt.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        poseSubAtt.topic.topicDataType = poseStampedPubSubType.getName(); //Must be registered before the creation of the subscriber
        poseSubAtt.topic.topicName = "rt/odom";
        pose_subscriber = eprosima::fastrtps::Domain::createSubscriber(fastrtps_participant, poseSubAtt, static_cast<eprosima::fastrtps::SubscriberListener *>(poseStampedListener));

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
        twist_publisher = eprosima::fastrtps::Domain::createPublisher(fastrtps_participant, Wparam, twistPubListener);
    }

private:
    void battery_update(float voltage)
    {
        auto message = sensor_msgs::msg::BatteryState();
        message.voltage = voltage;
        batt_pub_->publish(message);
    }

    void pose_update(eprosima::geometry_msgs::msg::PoseStamped fastrtps_pose)
    {
        auto cyclonedds_pose = geometry_msgs::msg::PoseStamped();
        cyclonedds_pose.pose.position.x = fastrtps_pose.pose().position().x();
        cyclonedds_pose.pose.position.y = fastrtps_pose.pose().position().y();
        cyclonedds_pose.pose.position.z = fastrtps_pose.pose().position().z();
        cyclonedds_pose.pose.orientation.x = fastrtps_pose.pose().orientation().x();
        cyclonedds_pose.pose.orientation.y = fastrtps_pose.pose().orientation().y();
        cyclonedds_pose.pose.orientation.z = fastrtps_pose.pose().orientation().z();
        cyclonedds_pose.pose.orientation.w = fastrtps_pose.pose().orientation().w();
        pose_pub_->publish(cyclonedds_pose);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linearMsg.x(msg->linear.x);
        angularMsg.z(msg->angular.z);
        twistMsg.linear(linearMsg);
        twistMsg.angular(angularMsg);
        twist_publisher->write((void *)&twistMsg);
    }

    eprosima::fastrtps::Participant *fastrtps_participant;
    eprosima::fastrtps::Subscriber *battery_subscriber;
    eprosima::fastrtps::Subscriber *pose_subscriber;
    eprosima::fastrtps::Publisher *twist_publisher;
    eprosima::sensor_msgs::msg::BatteryStatePubSubType batteryStatePubSubType;
    eprosima::geometry_msgs::msg::PoseStampedPubSubType poseStampedPubSubType;
    eprosima::geometry_msgs::msg::TwistPubSubType twistPubSubType;
    eprosima::geometry_msgs::msg::Vector3 linearMsg;
    eprosima::geometry_msgs::msg::Vector3 angularMsg;
    eprosima::geometry_msgs::msg::Twist twistMsg;

    eprosima::BatteryStateListener *batteryStateListener;
    eprosima::PoseStampedListener *poseStampedListener;
    eprosima::fastrtps::PublisherListener *twistPubListener;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DDS_Bridge>());
    rclcpp::shutdown();
    return 0;
}