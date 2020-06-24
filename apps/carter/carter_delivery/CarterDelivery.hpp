/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <queue>
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/state_machine/state_machine.hpp"
#include "messages/chat_message.capnp.h"
#include "messages/differential_base.capnp.h"

namespace isaac {

namespace behavior_tree {
namespace deprecated {
class GroupSelectorBehavior;
}  // deprecated
}  // behavior_trees

namespace navigation {
class WaypointMapLayer;
}  // navigation

namespace map {
class WaypointMapLayer;
}  // map

// This is the central state machine for the Carter delivery demo. It sends Carter to various
// waypoints based on instructions received via Slack.
class CarterDelivery : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // Messages received from slack which instruct carter what to do next
  ISAAC_PROTO_RX(ChatMessageProto, incoming_message);
  // Feedback about our progress towards the goal
  ISAAC_PROTO_RX(Goal2FeedbackProto, goal_feedback);
  // Messages sent back to the slack bot to give feedback
  ISAAC_PROTO_TX(ChatMessageProto, outgoing_message);
  // The desired targer waypoint where the robot wants to drive next
  ISAAC_PROTO_TX(GoalWaypointProto, target_waypoint);

  // Slack ID of operator. Using this ID, Carter shares updates with the operator and accept
  // special commands from the operator.
  ISAAC_PARAM(std::string, operator_slack_user);
  // Slack channel of operator. Using this channel, Carter shares updates with the operator and
  // accept special commands from the operator.
  ISAAC_PARAM(std::string, operator_slack_channel);
  // Slack ID of server. Carter asks server for an item using this ID.
  ISAAC_PARAM(std::string, server_slack_user);
  // Slack channel of server. Carter asks server for an item using this channel.
  ISAAC_PARAM(std::string, server_slack_channel);

  // The waypoint map layer with all valid waypoints, used to check if requests are valid.
  ISAAC_PARAM(std::string, waypoint_map_layer, "map/waypoints");

  // Reply message if carter does not understand the order.
  ISAAC_PARAM(std::string, help_message,
              "I could not understand your message. "
              "Please send 'bring me popcorn to MEETING ROOM' or "
              "'bring me popcorn to R1 CUBE NUMBER'.");

  // The state machine stage at which to start (see `State` below).
  ISAAC_PARAM(std::string, start_state, "kMaintenance");

  // The map waypoint which is used for picking up popcorn
  ISAAC_PARAM(std::string, popcorn_pickup_waypoint, "popcorn_pickup");

  // Parameter to get navigation mode behavior
  ISAAC_PARAM(std::string, navigation_mode,
      "navigation_mode/isaac.navigation.GroupSelectorBehavior");

 private:
  using State = std::string;

  // A message
  struct ChatMessage {
    std::string user;
    std::string channel;
    std::string text;
  };

  // Reads goal feedback message and updates associated variables
  void processGoalFeedback();

  // Gets the given state of the robot as a conversational string
  static std::string ToConversationalString(const State& state);

  // Creates the state machine
  void createStateMachine();

  // Has conversation with the robot
  void conversation();

  // Sends a chat message to the current customer
  void sendMessageToCustomer(std::string text);
  // Sends a chat message to the operator
  void sendMessageToOperator(std::string text);
  // Sends a chat message to the server
  void sendMessageToServer(std::string text);
  // Replies to the last message using text provided
  void sendReplyMessage(std::string text);

  // Sends a message to drive to the given waypoint
  void driveToWaypoint(const std::string& waypoint);

  int64_t goal_timestamp_;
  bool has_arrived_;
  std::optional<double> maybe_remaining_distance_;

  map::WaypointMapLayer* waypoint_map_layer_;
  behavior_tree::deprecated::GroupSelectorBehavior* navigation_mode_;

  bool printed_is_arriving_;

  state_machine::StateMachine<State> machine_;
  bool want_to_stop_;

  std::string desired_waypoint_;
  std::string desired_item_;
  std::string pickup_waypoint_;

  std::optional<ChatMessage> last_user_message_;
  std::optional<std::string> customer_user_;
  std::optional<std::string> customer_channel_;
  std::queue<ChatMessage> outbox_;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::CarterDelivery);
