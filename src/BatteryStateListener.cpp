#include "BatteryStateListener.h"

eprosima::BatteryStateListener::BatteryStateListener(std::function<void(eprosima::sensor_msgs::msg::BatteryState)> cb)
{
    callback = cb;
    n_matched = 0;
}

void eprosima::BatteryStateListener::onSubscriptionMatched(eprosima::fastrtps::Subscriber *sub, eprosima::fastrtps::rtps::MatchingInfo &info)
{
    (void)sub;

    if (info.status == eprosima::fastrtps::rtps::MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "BatteryState subscriber matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "BatteryState subscriber unmatched" << std::endl;
    }
}

void eprosima::BatteryStateListener::onNewDataMessage(eprosima::fastrtps::Subscriber *sub)
{
    // Take data
    eprosima::sensor_msgs::msg::BatteryState batteryState;

    if (sub->takeNextData(&batteryState, &m_info))
    {
        if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE)
        {
            callback(batteryState);
        }
    }
}