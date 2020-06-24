/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Mission.hpp"

#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/gems/serialization/json.hpp"
#include "messages/uuid.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace behavior_tree {

void Mission::start() {
  if (getNumChildren() != 1) {
    reportFailure("Mission only works with exactly one child. Got %lld", getNumChildren());
    return;
  }

  tickPeriodically();
}

void Mission::tick() {
  // Check if currently running mission has completed;
  if (current_mission_) {
    switch (getChildStatus(0)) {
      case alice::Status::SUCCESS:
        endCurrentMission(true);
        break;
      case alice::Status::FAILURE:
        endCurrentMission(false);
        break;
      case alice::Status::RUNNING:
        sendStatus(MissionStatusProto::MissionStatus::RUNNING);
        break;
      default:
        reportFailure("Behavior tree is in invalid state");
        return;
    }
  }

  rx_mission().processAllNewMessages([this] (auto proto, int64_t pubtime, int64_t acqtime) {
    startMission(proto);
  });
}

void Mission::startMission(const MissionProto::Reader& mission) {
  // If a mission is already running, end it with failure
  if (current_mission_) {
    endCurrentMission(false);
  }
  current_mission_ = alice::FromProto(mission.getUuid());

  // Attempt to set the application configuration in the MissionProto
  auto config_json = isaac::serialization::ParseJson(mission.getConfig().getSerialized());
  if (!config_json) {
    endCurrentMission(false);
    return;
  }
  node()->app()->backend()->config_backend()->set(*config_json);

  // Start the associated behavior
  startChild(0);

  // Send a message to indicate the mission has started
  sendStatus(MissionStatusProto::MissionStatus::RUNNING);
}

void Mission::endCurrentMission(bool success) {
  ASSERT(current_mission_, "Cannot end mission when no mission is running")
  stopChild(0);
  sendStatus(success ? MissionStatusProto::MissionStatus::SUCCESS :
             MissionStatusProto::MissionStatus::FAILURE);
  current_mission_ = std::nullopt;
}

void Mission::sendStatus(MissionStatusProto::MissionStatus status) {
  ASSERT(current_mission_, "Cannot send mission status with no mission running")
  auto status_proto = tx_mission_status().initProto();
  alice::ToProto(*current_mission_, status_proto.initUuid());
  status_proto.setMissionStatus(status);
  tx_mission_status().publish();
}

}  // namespace behavior_tree
}  // namespace isaac
