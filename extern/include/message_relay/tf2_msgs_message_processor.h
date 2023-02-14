// Generated by message_relay for processing frame IDs in tf2_msgs messages and services.
// DO NOT EDIT


#ifndef MESSAGE_RELAY_TF2_MSGS_MESSAGE_PROCESSOR_H
#define MESSAGE_RELAY_TF2_MSGS_MESSAGE_PROCESSOR_H

#include "message_relay/processor/message_processor.h"

#include "tf2_msgs/TF2Error.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_msgs/LookupTransformAction.h"
#include "tf2_msgs/LookupTransformActionGoal.h"
#include "tf2_msgs/LookupTransformActionResult.h"
#include "tf2_msgs/LookupTransformActionFeedback.h"
#include "tf2_msgs/LookupTransformGoal.h"
#include "tf2_msgs/LookupTransformResult.h"
#include "tf2_msgs/LookupTransformFeedback.h"

#include "tf2_msgs/FrameGraph.h"

namespace message_relay
{

template<>
void MessageProcessor<tf2_msgs::LookupTransformFeedback, FrameIdProcessor>::processMessage(tf2_msgs::LookupTransformFeedback::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformActionResult, FrameIdProcessor>::processMessage(tf2_msgs::LookupTransformActionResult::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::TFMessage, FrameIdProcessor>::processMessage(tf2_msgs::TFMessage::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformGoal, FrameIdProcessor>::processMessage(tf2_msgs::LookupTransformGoal::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformActionGoal, FrameIdProcessor>::processMessage(tf2_msgs::LookupTransformActionGoal::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::TF2Error, FrameIdProcessor>::processMessage(tf2_msgs::TF2Error::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformActionFeedback, FrameIdProcessor>::processMessage(tf2_msgs::LookupTransformActionFeedback::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformResult, FrameIdProcessor>::processMessage(tf2_msgs::LookupTransformResult::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformAction, FrameIdProcessor>::processMessage(tf2_msgs::LookupTransformAction::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void ServiceProcessor<tf2_msgs::FrameGraph, FrameIdProcessor>::processRequest(tf2_msgs::FrameGraph::Request &req, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void ServiceProcessor<tf2_msgs::FrameGraph, FrameIdProcessor>::processResponse(tf2_msgs::FrameGraph::Response &res, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformFeedback, TimeProcessor>::processMessage(tf2_msgs::LookupTransformFeedback::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformActionResult, TimeProcessor>::processMessage(tf2_msgs::LookupTransformActionResult::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<tf2_msgs::TFMessage, TimeProcessor>::processMessage(tf2_msgs::TFMessage::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformGoal, TimeProcessor>::processMessage(tf2_msgs::LookupTransformGoal::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformActionGoal, TimeProcessor>::processMessage(tf2_msgs::LookupTransformActionGoal::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<tf2_msgs::TF2Error, TimeProcessor>::processMessage(tf2_msgs::TF2Error::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformActionFeedback, TimeProcessor>::processMessage(tf2_msgs::LookupTransformActionFeedback::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformResult, TimeProcessor>::processMessage(tf2_msgs::LookupTransformResult::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<tf2_msgs::LookupTransformAction, TimeProcessor>::processMessage(tf2_msgs::LookupTransformAction::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void ServiceProcessor<tf2_msgs::FrameGraph, TimeProcessor>::processRequest(tf2_msgs::FrameGraph::Request &req, TimeProcessor::ConstPtr &time_processor);

template<>
void ServiceProcessor<tf2_msgs::FrameGraph, TimeProcessor>::processResponse(tf2_msgs::FrameGraph::Response &res, TimeProcessor::ConstPtr &time_processor);

}  // namespace message_relay

#endif // MESSAGE_RELAY_TF2_MSGS_MESSAGE_PROCESSOR_H