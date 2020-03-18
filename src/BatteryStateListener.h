#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include "BatteryStatePubSubTypes.h"

namespace eprosima
{

class BatteryStateListener : public eprosima::fastrtps::SubscriberListener
{
public:
    BatteryStateListener(std::function<void(float)> cb);
    ~BatteryStateListener(){};
    void onSubscriptionMatched(eprosima::fastrtps::Subscriber *sub, eprosima::fastrtps::rtps::MatchingInfo &info);
    void onNewDataMessage(eprosima::fastrtps::Subscriber *sub);
    eprosima::fastrtps::SampleInfo_t m_info;
    int n_matched;
    int n_msg;
    std::function<void(float)> callback;
};

}