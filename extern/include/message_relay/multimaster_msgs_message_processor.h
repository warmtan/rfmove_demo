// Generated by message_relay for processing frame IDs in multimaster_msgs messages and services.
// DO NOT EDIT


#ifndef MESSAGE_RELAY_MULTIMASTER_MSGS_MESSAGE_PROCESSOR_H
#define MESSAGE_RELAY_MULTIMASTER_MSGS_MESSAGE_PROCESSOR_H

#include "message_relay/processor/message_processor.h"

#include "multimaster_msgs/ClockOffset.h"

#include "multimaster_msgs/GetClockOffset.h"

namespace message_relay
{

template<>
void MessageProcessor<multimaster_msgs::ClockOffset, FrameIdProcessor>::processMessage(multimaster_msgs::ClockOffset::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void ServiceProcessor<multimaster_msgs::GetClockOffset, FrameIdProcessor>::processRequest(multimaster_msgs::GetClockOffset::Request &req, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void ServiceProcessor<multimaster_msgs::GetClockOffset, FrameIdProcessor>::processResponse(multimaster_msgs::GetClockOffset::Response &res, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<multimaster_msgs::ClockOffset, TimeProcessor>::processMessage(multimaster_msgs::ClockOffset::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void ServiceProcessor<multimaster_msgs::GetClockOffset, TimeProcessor>::processRequest(multimaster_msgs::GetClockOffset::Request &req, TimeProcessor::ConstPtr &time_processor);

template<>
void ServiceProcessor<multimaster_msgs::GetClockOffset, TimeProcessor>::processResponse(multimaster_msgs::GetClockOffset::Response &res, TimeProcessor::ConstPtr &time_processor);

}  // namespace message_relay

#endif // MESSAGE_RELAY_MULTIMASTER_MSGS_MESSAGE_PROCESSOR_H
