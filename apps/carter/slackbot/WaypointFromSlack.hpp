/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/optional.hpp"
#include "messages/chat_message.capnp.h"
#include "messages/differential_base.capnp.h"

namespace isaac {

namespace behavior_tree {
namespace deprecated {
class GroupSelectorBehavior;
}  // deprecated
}  // behavior_trees

namespace map {
class WaypointMapLayer;
}  // map

// Navigate to the waypoint received through slack
class WaypointFromSlack : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Messages received from slack which instruct carter what to do next
  ISAAC_PROTO_RX(ChatMessageProto, incoming_message);
  // Feedback about where we are with respect to the goal
  ISAAC_PROTO_RX(Goal2FeedbackProto, feedback);
  // Messages sent back through slack
  ISAAC_PROTO_TX(ChatMessageProto, outgoing_message);
  // The desired target waypoint where the robot wants to drive next
  ISAAC_PROTO_TX(GoalWaypointProto, target_waypoint);

  // The waypoint map layer with all valid waypoints. It is used to check if requests are valid.
  ISAAC_PARAM(std::string, waypoint_map_layer, "map/waypoints");
  // Parameter to get navigation mode behavior
  ISAAC_PARAM(std::string, navigation_mode,
      "navigation_mode/isaac.navigation.GroupSelectorBehavior");

 private:
  // Full stop
  void stopDriving();

  // Send a message to drive to the given waypoint
  void driveToWaypoint(const std::string& waypoint);

  // Send a message through slack
  void sendSlackMessage(const std::string& user, const std::string& channel,
                        const std::string& text);

  // To check whether a waypoint exists
  map::WaypointMapLayer* waypoint_map_layer_;

  // To start/stop navigation
  behavior_tree::deprecated::GroupSelectorBehavior* navigation_mode_;

  // Timestamp of the last goal
  int64_t goal_acqtime_;
  // Whether we are ready to take orders
  bool busy_;
  // Name of the last waypoint
  std::string waypoint_;

  // Customer is who we are driving for
  std::optional<std::string> customer_user_;
  std::optional<std::string> customer_channel_;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::WaypointFromSlack);
