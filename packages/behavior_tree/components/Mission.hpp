/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "messages/mission.capnp.h"
#include "packages/behavior_tree/components/Behavior.hpp"

namespace isaac {
namespace behavior_tree {

// @experimental
// This runs a mission when it receives a MissionProto message and reports back the status with
// MissionStatusProto. It starts running a new mission with the following steps:
// - If there is an older mission running, stop it and report it as failed
// - Set the application configuration specified in MissionProto
// - Start the child behavior tree
// - Send a MissionStatusProto indicating that the mission is running
// - When the child behavior tree finishes, send a MissionStatusProto with the mission status.
class Mission : public Behavior {
 public:
  void start() override;
  void tick() override;

  // When a message is received on this channel, start a mission
  ISAAC_PROTO_RX(MissionProto, mission);
  // This channel is used to report the status of a mission
  ISAAC_PROTO_TX(MissionStatusProto, mission_status);

 private:
  // Stops the behavior tree and sends a MissionStatusProto to indicate success or failure
  void endCurrentMission(bool success);
  // Starts the mission encoded in the given MissionProto
  void startMission(const MissionProto::Reader& mission);
  // Sends the current mission status
  void sendStatus(MissionStatusProto::MissionStatus status);

  // The currently running mission or std::nullopt if there is none
  std::optional<Uuid> current_mission_;
  // The node to trigger/monitor
  alice::Node* behavior_;
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::Mission);
