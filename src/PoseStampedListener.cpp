#include "PoseStampedListener.h"

eprosima::PoseStampedListener::PoseStampedListener(std::function<void(eprosima::geometry_msgs::msg::PoseStamped)> cb)
{
    callback = cb;
    n_matched = 0;
    n_msg = 0;
}

void eprosima::PoseStampedListener::onSubscriptionMatched(eprosima::fastrtps::Subscriber *sub, eprosima::fastrtps::rtps::MatchingInfo &info)
{
    (void)sub;

    if (info.status == eprosima::fastrtps::rtps::MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "PoseStamped subscriber matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "PoseStamped Subscriber unmatched" << std::endl;
    }
}

void eprosima::PoseStampedListener::onNewDataMessage(eprosima::fastrtps::Subscriber *sub)
{
    // Take data
    eprosima::geometry_msgs::msg::PoseStamped poseStamped;

    if (sub->takeNextData(&poseStamped, &m_info))
    {
        if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE)
        {
            callback(poseStamped);
        }
    }
}