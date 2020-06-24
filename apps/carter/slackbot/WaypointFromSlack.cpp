/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "WaypointFromSlack.hpp"

#include <string>

#include "engine/gems/algorithm/string_utils.hpp"
#include "packages/behavior_tree/components/deprecated/GroupSelectorBehavior.hpp"
#include "packages/map/WaypointMapLayer.hpp"

namespace isaac {

void WaypointFromSlack::start() {
  waypoint_map_layer_ =
      node()->app()->findComponentByName<map::WaypointMapLayer>(get_waypoint_map_layer());
  ASSERT(waypoint_map_layer_, "Could not find waypoint layer");

  navigation_mode_ = node()->app()->findComponentByName<
      behavior_tree::deprecated::GroupSelectorBehavior>(get_navigation_mode());
  ASSERT(navigation_mode_, "Could not find navigation mode");

  busy_ = false;
  show("busy", 0.0);
  goal_acqtime_ = 0;
  customer_user_ = std::nullopt;
  customer_channel_ = std::nullopt;

  tickPeriodically();
}

void WaypointFromSlack::tick() {
  // Process all slack messages received
  rx_incoming_message().processAllNewMessages([&](const auto proto_in, int64_t, int64_t) {
    // TODO Make sure we don't reply to ourself
    const std::string user = proto_in.getUser();
    const std::string channel = proto_in.getChannel();
    const std::string text = ToLowerCase(proto_in.getText());

    if (busy_) {
      this->sendSlackMessage(user, channel, "Sorry, I am driving to " + waypoint_ + " right now.");
      return;
    }

    if (!waypoint_map_layer_->hasWaypoint(text)) {
      this->sendSlackMessage(user, channel, "Sorry, I don't know where " + text + " is.");
      return;
    }

    this->driveToWaypoint(text);
    customer_user_ = user;
    customer_channel_ = channel;
    this->sendSlackMessage(user, channel, "Ok. Driving to " + text + ".");
  });

  // Process feedback about our progress towards the goal
  rx_feedback().processAllNewMessages([&](auto proto, int64_t pubtime, int64_t acqtime) {
    // If busy_ is false, there is nothing to check.
    // Plus, if we did not start driving yet harArrived may return false.
    if (!busy_) {
      return;
    }
    // Use feedback associated with our goal
    if (goal_acqtime_ == acqtime && proto.getHasArrived()) {
      this->stopDriving();
    }
  });
}

void WaypointFromSlack::sendSlackMessage(const std::string& user, const std::string& channel,
                                   const std::string& text) {
  auto proto_out = tx_outgoing_message().initProto();
  proto_out.setUser(user);
  proto_out.setChannel(channel);
  proto_out.setText(text);
  tx_outgoing_message().publish();
}

void WaypointFromSlack::stopDriving() {
  navigation_mode_->async_set_desired_behavior("stop");
  ASSERT(!waypoint_.empty(), "Logic error");
  ASSERT(customer_user_, "Logic error");
  ASSERT(customer_channel_, "Logic error");
  this->sendSlackMessage(*customer_user_, *customer_channel_, "Arrived at " + waypoint_ + ".");
  waypoint_ = "";
  customer_user_ = std::nullopt;
  customer_channel_ = std::nullopt;
  busy_ = false;
  show("busy", 0.0);
}

void WaypointFromSlack::driveToWaypoint(const std::string& waypoint) {
  waypoint_ = waypoint;
  navigation_mode_->async_set_desired_behavior("navigate");

  // Store the time so that we only react to messages referring to this goal
  goal_acqtime_ = getTickTimestamp();
  busy_ = true;
  show("busy", 1.0);

  // Publish the target waypoint.
  auto proto = tx_target_waypoint().initProto();
  proto.setWaypoint(waypoint);
  tx_target_waypoint().publish(goal_acqtime_);
}

}  // namespace isaac
