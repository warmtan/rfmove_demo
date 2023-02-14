// Generated by message_relay for processing frame IDs in move_base_msgs messages and services.
// DO NOT EDIT


#ifndef MESSAGE_RELAY_MOVE_BASE_MSGS_MESSAGE_PROCESSOR_H
#define MESSAGE_RELAY_MOVE_BASE_MSGS_MESSAGE_PROCESSOR_H

#include "message_relay/processor/message_processor.h"

#include "move_base_msgs/RecoveryStatus.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "move_base_msgs/MoveBaseResult.h"
#include "move_base_msgs/MoveBaseFeedback.h"


namespace message_relay
{

template<>
void MessageProcessor<move_base_msgs::RecoveryStatus, FrameIdProcessor>::processMessage(move_base_msgs::RecoveryStatus::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseFeedback, FrameIdProcessor>::processMessage(move_base_msgs::MoveBaseFeedback::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseActionFeedback, FrameIdProcessor>::processMessage(move_base_msgs::MoveBaseActionFeedback::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseGoal, FrameIdProcessor>::processMessage(move_base_msgs::MoveBaseGoal::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseAction, FrameIdProcessor>::processMessage(move_base_msgs::MoveBaseAction::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseActionResult, FrameIdProcessor>::processMessage(move_base_msgs::MoveBaseActionResult::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseResult, FrameIdProcessor>::processMessage(move_base_msgs::MoveBaseResult::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseActionGoal, FrameIdProcessor>::processMessage(move_base_msgs::MoveBaseActionGoal::Ptr &msg, FrameIdProcessor::ConstPtr &frame_id_processor);

template<>
void MessageProcessor<move_base_msgs::RecoveryStatus, TimeProcessor>::processMessage(move_base_msgs::RecoveryStatus::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseFeedback, TimeProcessor>::processMessage(move_base_msgs::MoveBaseFeedback::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseActionFeedback, TimeProcessor>::processMessage(move_base_msgs::MoveBaseActionFeedback::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseGoal, TimeProcessor>::processMessage(move_base_msgs::MoveBaseGoal::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseAction, TimeProcessor>::processMessage(move_base_msgs::MoveBaseAction::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseActionResult, TimeProcessor>::processMessage(move_base_msgs::MoveBaseActionResult::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseResult, TimeProcessor>::processMessage(move_base_msgs::MoveBaseResult::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

template<>
void MessageProcessor<move_base_msgs::MoveBaseActionGoal, TimeProcessor>::processMessage(move_base_msgs::MoveBaseActionGoal::Ptr &msg, TimeProcessor::ConstPtr &time_processor);

}  // namespace message_relay

#endif // MESSAGE_RELAY_MOVE_BASE_MSGS_MESSAGE_PROCESSOR_H
