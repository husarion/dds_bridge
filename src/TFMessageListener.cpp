#include "TFMessageListener.h"

eprosima::TFMessageListener::TFMessageListener(std::function<void(eprosima::tf2_msgs::msg::TFMessage)> cb)
{
    callback = cb;
    n_matched = 0;
    n_msg = 0;
}

void eprosima::TFMessageListener::onSubscriptionMatched(eprosima::fastrtps::Subscriber *sub, eprosima::fastrtps::rtps::MatchingInfo &info)
{
    (void)sub;

    if (info.status == eprosima::fastrtps::rtps::MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "TFMessage subscriber matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "TFMessage subscriber unmatched" << std::endl;
    }
}

void eprosima::TFMessageListener::onNewDataMessage(eprosima::fastrtps::Subscriber *sub)
{
    // Take data
    eprosima::tf2_msgs::msg::TFMessage tfMessage;
    if (sub->takeNextData(&tfMessage, &m_info))
    {
        if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE)
        {
            callback(tfMessage);
        }
    }
}
