#include "BatteryStateListener.h"

eprosima::BatteryStateListener::BatteryStateListener(std::function<void(float)> cb)
{
    callback = cb;
    n_matched = 0;
    n_msg = 0;
}

void eprosima::BatteryStateListener::onSubscriptionMatched(eprosima::fastrtps::Subscriber *sub, eprosima::fastrtps::rtps::MatchingInfo &info)
{
    (void)sub;

    if (info.status == eprosima::fastrtps::rtps::MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber unmatched" << std::endl;
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
            // Print your structure data here.
            ++n_msg;
            // float v = ;
            // std::cout << "FastRTPS battery received, count=" << batteryState.voltage() << std::endl;
            callback(batteryState.voltage());
        }
    }
}