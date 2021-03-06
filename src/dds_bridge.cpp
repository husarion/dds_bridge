#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

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
#include "TFMessagePubSubTypes.h"
#include "TFMessageListener.h"

#include <fastrtps/Domain.h>

using namespace std::chrono_literals;

using std::placeholders::_1;

class DDS_Bridge : public rclcpp::Node
{
public:
    DDS_Bridge()
        : Node("dds_bridge")
    {
        batt_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery", 1);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/odom", 1);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // Create RTPSParticipant
        eprosima::fastrtps::ParticipantAttributes PParam;
        PParam.rtps.setName("fastrtps_participant_subscriber");
        fastrtps_participant = eprosima::fastrtps::Domain::createParticipant(PParam);

        // //Register the type
        eprosima::fastrtps::Domain::registerType(fastrtps_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&batteryStatePubSubType));
        eprosima::fastrtps::Domain::registerType(fastrtps_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&poseStampedPubSubType));
        eprosima::fastrtps::Domain::registerType(fastrtps_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&transformStampedPubSubType));
        eprosima::fastrtps::Domain::registerType(fastrtps_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&twistPubSubType));

        // Create Subscriber
        batteryStateListener = new eprosima::BatteryStateListener(std::bind(&DDS_Bridge::battery_update, this, _1));
        poseStampedListener = new eprosima::PoseStampedListener(std::bind(&DDS_Bridge::pose_update, this, _1));
        transformStampedListener = new eprosima::TFMessageListener(std::bind(&DDS_Bridge::transform_update, this, _1));
        twistPubListener = new eprosima::fastrtps::PublisherListener();

        eprosima::fastrtps::SubscriberAttributes batterySubAtt;
        batterySubAtt.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        batterySubAtt.topic.topicDataType = batteryStatePubSubType.getName(); //Must be registered before the creation of the subscriber
        batterySubAtt.topic.topicName = "rt/battery";
        battery_subscriber = eprosima::fastrtps::Domain::createSubscriber(fastrtps_participant, batterySubAtt, static_cast<eprosima::fastrtps::SubscriberListener *>(batteryStateListener));

        eprosima::fastrtps::SubscriberAttributes poseSubAtt;
        poseSubAtt.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        poseSubAtt.topic.topicDataType = poseStampedPubSubType.getName(); //Must be registered before the creation of the subscriber
        poseSubAtt.topic.topicName = "rt/odom";
        pose_subscriber = eprosima::fastrtps::Domain::createSubscriber(fastrtps_participant, poseSubAtt, static_cast<eprosima::fastrtps::SubscriberListener *>(poseStampedListener));

        eprosima::fastrtps::SubscriberAttributes transformSubAtt;
        transformSubAtt.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
        transformSubAtt.topic.topicDataType = transformStampedPubSubType.getName(); //Must be registered before the creation of the subscriber
        transformSubAtt.topic.topicName = "rt/tf";
        transform_subscriber = eprosima::fastrtps::Domain::createSubscriber(fastrtps_participant, transformSubAtt, static_cast<eprosima::fastrtps::SubscriberListener *>(transformStampedListener));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&DDS_Bridge::cmd_vel_callback, this, _1));

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
    void battery_update(eprosima::sensor_msgs::msg::BatteryState fastrtps_battery_state)
    {
        auto cyclonedds_battery_state = sensor_msgs::msg::BatteryState();
        cyclonedds_battery_state.header.frame_id = fastrtps_battery_state.header().frame_id();
        cyclonedds_battery_state.header.stamp.sec = fastrtps_battery_state.header().stamp().sec();
        cyclonedds_battery_state.header.stamp.nanosec = fastrtps_battery_state.header().stamp().nanosec();
        cyclonedds_battery_state.voltage = fastrtps_battery_state.voltage();
        cyclonedds_battery_state.temperature = fastrtps_battery_state.temperature();
        cyclonedds_battery_state.current = fastrtps_battery_state.current();
        cyclonedds_battery_state.charge = fastrtps_battery_state.charge();
        cyclonedds_battery_state.capacity = fastrtps_battery_state.capacity();
        cyclonedds_battery_state.design_capacity = fastrtps_battery_state.design_capacity();
        cyclonedds_battery_state.percentage = fastrtps_battery_state.percentage();
        cyclonedds_battery_state.power_supply_status = fastrtps_battery_state.power_supply_status();
        cyclonedds_battery_state.power_supply_health = fastrtps_battery_state.power_supply_health();
        cyclonedds_battery_state.power_supply_technology = fastrtps_battery_state.power_supply_technology();
        cyclonedds_battery_state.present = fastrtps_battery_state.present();
        cyclonedds_battery_state.cell_voltage = fastrtps_battery_state.cell_voltage();
        cyclonedds_battery_state.cell_temperature = fastrtps_battery_state.cell_temperature();
        cyclonedds_battery_state.location = fastrtps_battery_state.location();
        cyclonedds_battery_state.serial_number = fastrtps_battery_state.serial_number();
        batt_pub_->publish(cyclonedds_battery_state);
    }

    void pose_update(eprosima::geometry_msgs::msg::PoseStamped fastrtps_pose)
    {
        auto cyclonedds_pose = geometry_msgs::msg::PoseStamped();
        cyclonedds_pose.header.frame_id = fastrtps_pose.header().frame_id();
        cyclonedds_pose.header.stamp.sec = fastrtps_pose.header().stamp().sec();
        cyclonedds_pose.header.stamp.nanosec = fastrtps_pose.header().stamp().nanosec();
        cyclonedds_pose.pose.position.x = fastrtps_pose.pose().position().x();
        cyclonedds_pose.pose.position.y = fastrtps_pose.pose().position().y();
        cyclonedds_pose.pose.position.z = fastrtps_pose.pose().position().z();
        cyclonedds_pose.pose.orientation.x = fastrtps_pose.pose().orientation().x();
        cyclonedds_pose.pose.orientation.y = fastrtps_pose.pose().orientation().y();
        cyclonedds_pose.pose.orientation.z = fastrtps_pose.pose().orientation().z();
        cyclonedds_pose.pose.orientation.w = fastrtps_pose.pose().orientation().w();
        pose_pub_->publish(cyclonedds_pose);
    }

    void transform_update(eprosima::tf2_msgs::msg::TFMessage fastrtps_transform_message)
    {
        eprosima::geometry_msgs::msg::TransformStamped fastrtps_transform;
        for (auto i = 0u; i < fastrtps_transform_message.transforms().size(); i++)
        {
            if (fastrtps_transform_message.transforms()[i].child_frame_id() == "base_link" && fastrtps_transform_message.transforms()[i].header().frame_id() == "odom")
            {
                fastrtps_transform = fastrtps_transform_message.transforms()[i];
            }
        }
        geometry_msgs::msg::TransformStamped cyclonedds_transform;
        cyclonedds_transform.header.frame_id = fastrtps_transform.header().frame_id();
        cyclonedds_transform.header.stamp.sec = fastrtps_transform.header().stamp().sec();
        cyclonedds_transform.header.stamp.nanosec = fastrtps_transform.header().stamp().nanosec();
        cyclonedds_transform.child_frame_id = fastrtps_transform.child_frame_id();
        cyclonedds_transform.transform.translation.x = fastrtps_transform.transform().translation().x();
        cyclonedds_transform.transform.translation.y = fastrtps_transform.transform().translation().y();
        cyclonedds_transform.transform.translation.z = fastrtps_transform.transform().translation().z();
        cyclonedds_transform.transform.rotation.x = fastrtps_transform.transform().rotation().x();
        cyclonedds_transform.transform.rotation.y = fastrtps_transform.transform().rotation().y();
        cyclonedds_transform.transform.rotation.z = fastrtps_transform.transform().rotation().z();
        cyclonedds_transform.transform.rotation.w = fastrtps_transform.transform().rotation().w();
        tf_broadcaster->sendTransform(cyclonedds_transform);
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
    eprosima::fastrtps::Subscriber *transform_subscriber;
    eprosima::fastrtps::Publisher *twist_publisher;
    eprosima::sensor_msgs::msg::BatteryStatePubSubType batteryStatePubSubType;
    eprosima::geometry_msgs::msg::PoseStampedPubSubType poseStampedPubSubType;
    eprosima::tf2_msgs::msg::TFMessagePubSubType transformStampedPubSubType;
    eprosima::geometry_msgs::msg::TwistPubSubType twistPubSubType;
    eprosima::geometry_msgs::msg::Vector3 linearMsg;
    eprosima::geometry_msgs::msg::Vector3 angularMsg;
    eprosima::geometry_msgs::msg::Twist twistMsg;

    eprosima::BatteryStateListener *batteryStateListener;
    eprosima::PoseStampedListener *poseStampedListener;
    eprosima::TFMessageListener *transformStampedListener;
    eprosima::fastrtps::PublisherListener *twistPubListener;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DDS_Bridge>());
    rclcpp::shutdown();
    return 0;
}