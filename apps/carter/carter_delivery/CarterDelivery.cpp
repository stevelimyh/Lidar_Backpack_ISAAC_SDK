/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CarterDelivery.hpp"

#include <string>
#include <utility>
#include <vector>

#include "engine/gems/algorithm/string_utils.hpp"
#include "messages/math.hpp"
#include "packages/behavior_tree/components/deprecated/GroupSelectorBehavior.hpp"
#include "packages/behavior_tree/components/deprecated/SelectorBehavior.hpp"
#include "packages/map/WaypointMapLayer.hpp"

namespace isaac {

namespace {
// Stopwatch to update user about distance remaining
constexpr char kDistanceUpdateHelper[] = "distance-update-helper";
constexpr double kDistanceUpdateInterval = 30.0;

// Name of states
constexpr char kStateInit[] = "kInit";
constexpr char kStateExit[] = "kExit";
constexpr char kStateMaintenance[] = "kMaintenance";
constexpr char kStateIdle[] = "kIdle";
constexpr char kStateDriveToPickup[] = "kDriveToPickup";
constexpr char kStateWaitForPickup[] = "kWaitForPickup";
constexpr char kStateDriveToDropoff[] = "kDriveToDropoff";
constexpr char kStateWaitForDropoff[] = "kWaitForDropoff";

// Checks if a text is similar to the desired content
bool ContainsContents(const std::string& text, const std::string& content) {
  // If contents is contained in the text we are good
  if (text.find(content) != std::string::npos) {
    return true;
  }
  // Check that every word in content appears in order
  size_t start = 0;
  for (const auto& token : SplitString(content, ' ')) {
    const size_t next = text.find(token, start);
    if (next == std::string::npos) {
      return false;
    }
  }
  return true;
}

// Extracts waypoint and ordered item from the message text
bool ExtractOrder(const std::string& text, std::string& waypoint, std::string& item) {
  constexpr char kPopcornOrder[] = "bring me popcorn to ";
  if (StartsWith(text, kPopcornOrder)) {
    waypoint = ToLowerCase(text.substr(sizeof(kPopcornOrder) - 1));
    item = "popcorn";
    return true;
  } else {
    waypoint = "";
    item = "";
    return false;
  }
}

}  // namespace

void CarterDelivery::processGoalFeedback() {
  // We can't confirm arrival without feedback
  if (!rx_goal_feedback().available()) return;
  // To verify arrival, we need the feedback associated with our latest goal
  const int64_t feedback_timestamp = rx_goal_feedback().acqtime();
  if (feedback_timestamp != goal_timestamp_) {
    has_arrived_ = false;
    maybe_remaining_distance_ = std::nullopt;
    return;
  }
  auto feedback_proto = rx_goal_feedback().getProto();
  has_arrived_ = feedback_proto.getHasArrived();
  const Pose2d robot_T_goal = FromProto(feedback_proto.getRobotTGoal());
  maybe_remaining_distance_ = robot_T_goal.translation.norm();
}

void CarterDelivery::start() {
  waypoint_map_layer_ =
      node()->app()->findComponentByName<map::WaypointMapLayer>(get_waypoint_map_layer());
  ASSERT(waypoint_map_layer_, "Could not find waypoint layer");

  navigation_mode_ = node()->app()->findComponentByName<
      behavior_tree::deprecated::GroupSelectorBehavior>(get_navigation_mode());
  ASSERT(navigation_mode_, "Could not find navigation mode");

  want_to_stop_ = false;
  goal_timestamp_ = 0;
  has_arrived_ = false;
  maybe_remaining_distance_ = std::nullopt;

  createStateMachine();

  // Start in desired state
  machine_.start(get_start_state());
  stopwatch(kDistanceUpdateHelper).setClock(node()->clock());
  tickPeriodically();
}

void CarterDelivery::tick() {
  processGoalFeedback();
  bool ticked_state_machine = false;
  // Update latest slack message
  rx_incoming_message().processAllNewMessages([&](const auto proto, int64_t, int64_t) {
    const std::string user = proto.getUser();
    // TODO Make sure we don't reply to ourself
    last_user_message_ = ChatMessage{user, proto.getChannel(), ToLowerCase(proto.getText())};
    // tick the state machine for every new message
    machine_.tick();
    this->conversation();
    ticked_state_machine = true;
  });
  // tick the state machine in case we did not receive a message
  last_user_message_ = std::nullopt;
  if (!ticked_state_machine) {
    machine_.tick();
  }
  // TODO: Show the current state in sight. We have a string though..
  // Send potential messages
  while (!outbox_.empty()) {
    auto proto = tx_outgoing_message().initProto();
    proto.setUser(outbox_.front().user);
    proto.setChannel(outbox_.front().channel);
    proto.setText(outbox_.front().text);
    outbox_.pop();
    tx_outgoing_message().publish();
  }
}

void CarterDelivery::stop() {
  // One last tick to get go into the exit state
  want_to_stop_ = true;
  machine_.tick();
  ASSERT(machine_.current_state() && *machine_.current_state() == kStateExit, "logic error");
  // Then we can stop the state machine properly
  machine_.stop();
}

std::string CarterDelivery::ToConversationalString(const State& state) {
  // TODO Write better messages
  return "I am currently in the state: '" + state.substr(1) + "'";
}

void CarterDelivery::createStateMachine() {
  const std::vector<State> all_states = {
      kStateIdle,
      kStateDriveToPickup,
      kStateWaitForPickup,
      kStateDriveToDropoff,
      kStateWaitForDropoff
  };

  machine_.setToString([this] (const State& state) { return state; });

  // The robot always goes first in the initial state
  machine_.addState(kStateInit,
      [this] {
        driveToWaypoint("");
        sendMessageToOperator("Hello, this is Carter the delivery robot.");
      },
      [] {}, [] {});

  // When the codelet stops the robot goes into the exit state
  machine_.addState(kStateExit,
      [this] {
        driveToWaypoint("");
        sendMessageToOperator("I am shutting down. See you next time!");
      },
      [] {}, [] {});

  // Idle state for waiting for new orders
  machine_.addState(kStateIdle,
      [this] {
        sendMessageToOperator("I am waiting for orders..");
      },
      [] {}, [] {});

  // Transitions from initialization
  machine_.addTransition(kStateInit, kStateIdle,
                         [this] { return true; }, [] {});

  machine_.addState(kStateDriveToPickup,
      [this] {
        sendMessageToOperator("Driving to waypoint " + pickup_waypoint_);
        driveToWaypoint(pickup_waypoint_);
      },
      [] { },
      [this] {
        driveToWaypoint("");
        pickup_waypoint_ = "";
      });

  machine_.addTransition(kStateIdle, kStateDriveToPickup,
      [this] {
        if (!last_user_message_) {
          return false;
        }
        // Do not interfere with operator command
        if (last_user_message_->text == "stop") {
          return false;
        }
        const bool ok = ExtractOrder(last_user_message_->text, desired_waypoint_, desired_item_);
        if (!ok) {
          sendReplyMessage(get_help_message());
          return false;
        }
        if (!waypoint_map_layer_->hasWaypoint(desired_waypoint_)) {
          sendReplyMessage("I do not know the meeting room '" + desired_waypoint_ + "'.");
          desired_waypoint_ = "";
          desired_item_ = "";
          return false;
        }
        if (desired_item_ == "popcorn") {
          pickup_waypoint_ = get_popcorn_pickup_waypoint();
        } else {
          ASSERT("Invalid item '%s'", desired_item_.c_str());
        }
        if (!waypoint_map_layer_->hasWaypoint(pickup_waypoint_)) {
          sendReplyMessage("I am not ready to pickup '" + desired_item_ +
                           "' yet. Please try again later.");
          sendMessageToOperator("No waypoint named '" + pickup_waypoint_ +
                                "' is defined in the map!");
          desired_waypoint_ = "";
          desired_item_ = "";
          return false;
        }
        return true;
      },
      [this] {
        customer_user_ = last_user_message_->user;
        customer_channel_ = last_user_message_->channel;
        sendMessageToCustomer("Received your order for '" + desired_item_ + "' to '" +
                              desired_waypoint_ + "'.");
      });

  machine_.addState(kStateWaitForPickup,
      [this] {
        sendMessageToCustomer("Waiting for server to place your " + desired_item_ +
                              " on the robot");
        sendMessageToServer("Please put " + desired_item_ + " on Carter");
      },
      [] { },
      [] { });

  machine_.addTransition(kStateDriveToPickup, kStateWaitForPickup,
      [this] {
        if (has_arrived_) {
          sendMessageToOperator("Arrived at waypoint " + pickup_waypoint_);
          return true;
        }
        return false;
      },
      [] { });

  machine_.addState(
      kStateDriveToDropoff,
      [this] {
        sendMessageToOperator("Driving to waypoint " + desired_waypoint_);
        sendMessageToCustomer("Your delivery is on its way!");
        driveToWaypoint(desired_waypoint_);
        printed_is_arriving_ = false;
      },
      [this] {
        if (stopwatch(kDistanceUpdateHelper).interval(kDistanceUpdateInterval)) {
          // Check if we have received an associated goal feedback yet.
          if (!maybe_remaining_distance_) return;
          const int distance_int = 5 * static_cast<int>(0.2 * *maybe_remaining_distance_);
          if (distance_int <= 10) {
            if (printed_is_arriving_) {
              sendMessageToCustomer("Your item is arriving! "
                                    "Please make sure the space in front of the meeting room is "
                                    "not blocked and wait until I came to a complete stop. "
                                    "I will notify you to pickup the item once I am ready.");
              printed_is_arriving_ = true;
            }
          } else {
            sendMessageToCustomer("Your item is about " + std::to_string(distance_int)
                                  + " meters away");
          }
        }
      },
      [this] {
        driveToWaypoint("");
      });

  machine_.addTransition(kStateWaitForPickup, kStateDriveToDropoff,
      [this] {
        if (desired_item_ != "popcorn") {
          return false;
        }
        // The user sends a message which instructs the robot to bring him a salad.
        if (!last_user_message_) {
          return false;
        }

        // Accept message only from the server or operator
        const bool from_server = last_user_message_->user == get_server_slack_user() &&
                                 last_user_message_->channel == get_server_slack_channel();
        const bool from_operator = last_user_message_->user == get_operator_slack_user() &&
                                   last_user_message_->channel == get_operator_slack_channel();
        if (!from_server && !from_operator) {
          return false;
        }

        // TODO: Check who sent the message
        if (!ContainsContents(last_user_message_->text, "ready")) {
          return false;
        }
        last_user_message_ = std::nullopt;
        return true;
      },
      [] { });

  machine_.addState(kStateWaitForDropoff,
      [this] {
        sendMessageToCustomer("Please pickup your item in front of meeting room '" +
                              desired_waypoint_ +"'. Reply 'ready' once you picked it up.");
      },
      [] { },
      [this] {
        sendMessageToCustomer("Thank you for using Carter! See you again soon");
        customer_user_ = std::nullopt;
        customer_channel_ = std::nullopt;
      });

  machine_.addTransition(kStateDriveToDropoff, kStateWaitForDropoff,
      [this] {
        return has_arrived_;
      },
      [] { });

  machine_.addTransition(kStateWaitForDropoff, kStateIdle,
      [this] {
        // The user sends a message which instructs the robot to bring him a salad.
        if (!last_user_message_) {
          return false;
        }

        // Accept message only from the customer or operator
        ASSERT(customer_user_, "No customer");
        ASSERT(customer_channel_, "No customer");
        const bool from_customer = last_user_message_->user == customer_user_ &&
                                   last_user_message_->channel == customer_channel_;
        const bool from_operator = last_user_message_->user == get_operator_slack_user() &&
                                   last_user_message_->channel == get_operator_slack_channel();
        if (!from_customer && !from_operator) {
          return false;
        }

        if (!ContainsContents(last_user_message_->text, "ready")) {
          return false;
        }
        last_user_message_ = std::nullopt;
        return true;
      },
      [] { });

  // Allow to stop from all states (expect exit state itself..)
  for (State state : all_states) {
    machine_.addTransition(state, kStateExit, [this] { return want_to_stop_; }, [] {});
  }

  // Operator override mode
  machine_.addState(kStateMaintenance,
      [this] {
        sendMessageToOperator("Entered manual mode");
      },
      [] { },
      [this] {
        sendMessageToOperator("Stopped manual mode");
      });
  for (State state : all_states) {
    machine_.addTransition(state, kStateMaintenance,
        [this] {
          const bool stop = last_user_message_
              && last_user_message_->user == get_operator_slack_user()
              && last_user_message_->channel == get_operator_slack_channel()
              && last_user_message_->text == "stop";
          if (!stop) {
            return false;
          }
          last_user_message_ = std::nullopt;
          return true;
        },
        [] { });
  }
  machine_.addTransition(kStateMaintenance, kStateIdle,
      [this] {
        const bool start = last_user_message_
            && last_user_message_->user == get_operator_slack_user()
            && last_user_message_->channel == get_operator_slack_channel()
            && last_user_message_->text == "start";
        if (!start) {
          return false;
        }
        last_user_message_ = std::nullopt;
        return true;
      },
      [] { });
  machine_.addTransition(kStateMaintenance, kStateExit, [this] { return want_to_stop_; }, [] {});
}

void CarterDelivery::conversation() {
  if (!last_user_message_) {
    return;
  }
  if (ContainsContents(last_user_message_->text, "hello")) {
    sendReplyMessage("Hello, how are you?");
  } else if (ContainsContents(last_user_message_->text, "status")) {
    if (!machine_.current_state()) {
      sendReplyMessage("I don't know what's happening...");
    } else {
      sendReplyMessage(ToConversationalString(*machine_.current_state()));
    }
  } else if (ContainsContents(last_user_message_->text, "whoami")) {
    sendReplyMessage("You are user " + last_user_message_->user + " on channel "
                     + last_user_message_->channel + ".");
  } else {
    if (machine_.current_state() && machine_.current_state().value() == kStateMaintenance) {
      // Notify potential customers that we are in maintenance mode
      sendReplyMessage("I am currently in maintenance mode. Please try again later.");
    } else if (customer_user_ && last_user_message_->user != customer_user_) {
      // If we have a customer and someone else talks to us, reject the latter.
      sendReplyMessage("I am already delivering and order. Please try again.");
    } else if (customer_user_ && last_user_message_->user == customer_user_) {
      // If the customers talks to us, ask him to be patient.
      sendReplyMessage("Please wait a bit. I will message you once I am ready.");
    } else {
      // TODO Handle more cases and reply something smart.
      sendReplyMessage("Just a second!");
    }
  }
}

void CarterDelivery::sendMessageToCustomer(std::string text) {
  if (!customer_user_ || !customer_channel_) {
    LOG_ERROR("There is currently no customer to which a message could be sent.");
    return;
  }
  sendMessageToOperator("Message to customer (" + *customer_user_ + "): " + text);
  outbox_.push(ChatMessage{*customer_user_, *customer_channel_, std::move(text)});
  // Clear the last user message to avoid sending unnecessary replies in conversation()
  last_user_message_ = std::nullopt;
}

void CarterDelivery::sendMessageToOperator(std::string text) {
  outbox_.push(ChatMessage{get_operator_slack_user(), get_operator_slack_channel(),
                           std::move(text)});
  // Clear the last user message to avoid sending unnecessary replies in conversation()
  last_user_message_ = std::nullopt;
}

void CarterDelivery::sendMessageToServer(std::string text) {
  sendMessageToOperator("Message to server (" + get_server_slack_user() + "): " + text);
  outbox_.push(ChatMessage{get_server_slack_user(), get_server_slack_channel(), std::move(text)});
  // Clear the last user message to avoid sending unnecessary replies in conversation()
  last_user_message_ = std::nullopt;
}

void CarterDelivery::sendReplyMessage(std::string text) {
  if (!last_user_message_) {
    LOG_ERROR("Could not send reply message as there is no last message");
  }
  sendMessageToOperator("Message to other (" + last_user_message_->user + "): " + text);
  outbox_.push(ChatMessage{last_user_message_->user, last_user_message_->channel, std::move(text)});
  // Clear the last user message to avoid sending unnecessary replies in conversation()
  last_user_message_ = std::nullopt;
}

void CarterDelivery::driveToWaypoint(const std::string& waypoint) {
  // If we don't have a waypoint we want to complete disable robot motion. This is for example
  // important so that the customer can take the popcorn without the robot baking up.
  navigation_mode_->async_set_desired_behavior(waypoint.empty() ? "stop" : "navigate");

  // Publish the target waypoint.
  goal_timestamp_ = getTickTimestamp();
  auto proto = tx_target_waypoint().initProto();
  proto.setWaypoint(waypoint);
  tx_target_waypoint().publish(goal_timestamp_);
}

}  // namespace isaac
