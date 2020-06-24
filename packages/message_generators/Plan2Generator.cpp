/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Plan2Generator.hpp"

#include <string>
#include <vector>

#include "messages/math.hpp"

namespace isaac {
namespace message_generators {

namespace {
// Color to be used while showing plan in Sight
const char kColor[] = "#f00";
}  // namespace

void Plan2Generator::start() {
  last_frame_ = std::nullopt;
  last_waypoints_ = std::nullopt;
  last_static_T_plan_ = std::nullopt;
  tickPeriodically();
}

void Plan2Generator::tick() {
  // Read parameters
  const std::string plan_frame = get_plan_frame();
  const std::string static_frame = get_static_frame();
  const std::optional<std::vector<std::string>> maybe_frames = try_get_frames();
  std::optional<std::vector<Pose2d>> maybe_waypoints = try_get_waypoints();
  if (plan_frame.empty()) {
    reportFailure("Empty plan frame");
    return;
  }
  if (static_frame.empty()) {
    reportFailure("Empty static frame");
    return;
  }
  if (static_cast<bool>(maybe_waypoints) == static_cast<bool>(maybe_frames)) {
    reportFailure("Either 'waypoints' or 'frames' parameter should be set");
    return;
  }

  // Read from Pose Tree
  const auto maybe_static_T_plan =
      node()->pose().tryGetPose2XY(static_frame, plan_frame, getTickTime());
  if (!maybe_static_T_plan) {
    reportFailure("Failed to read %s_T_%s from Pose Tree", static_frame.c_str(),
                  plan_frame.c_str());
    return;
  }
  if (maybe_frames) {
    std::vector<Pose2d> waypoints;
    for (const std::string& frame : *maybe_frames) {
      const auto maybe_plan_T_frame =
          node()->pose().tryGetPose2XY(plan_frame, frame, getTickTime());
      if (!maybe_plan_T_frame) {
        return;
      }
      waypoints.emplace_back(*maybe_plan_T_frame);
    }
    maybe_waypoints = waypoints;
  }

  // Do not repeat the same plan
  if (!isPlanNew(plan_frame, *maybe_static_T_plan, *maybe_waypoints)) {
    return;
  }
  last_frame_ = plan_frame;
  last_waypoints_ = *maybe_waypoints;
  last_static_T_plan_ = *maybe_static_T_plan;

  // Publish plan
  auto proto = tx_plan().initProto();
  const size_t num_poses = maybe_waypoints->size();
  auto poses_proto = proto.initPoses(num_poses);
  for (size_t i = 0; i < num_poses; i++) {
    ToProto(maybe_waypoints.value()[i], poses_proto[i]);
  }
  proto.setPlanFrame(plan_frame);
  tx_plan().publish();

  // visualize
  show("path", [&](sight::Sop& sop) {
    // show path
    sop.transform = sight::SopTransform{plan_frame};
    sop.style = sight::SopStyle{kColor, true};
    std::vector<Vector2d> path;
    for (const Pose2d& waypoint : *maybe_waypoints) {
      path.push_back(waypoint.translation);
    }
    sop.add(path);
  });
}

void Plan2Generator::stop() {
  show("path", [&](sight::Sop& sop) {});
}

bool Plan2Generator::isPlanNew(const std::string& plan_frame, const Pose2d& static_T_plan,
                               const std::vector<Pose2d>& waypoints) {
  if (!last_frame_) {
    // We did not send a plan before.
    return true;
  }
  ASSERT(last_static_T_plan_, "last_static_T_plan_ is null while last_frame_ is not");
  ASSERT(last_waypoints_, "last_waypoints_ is null while last_frame_ is not");

  if (*last_frame_ != plan_frame) {
    return true;
  }

  if (!arePosesWithinThreshold(*last_static_T_plan_, static_T_plan)) {
    return true;
  }

  if (last_waypoints_->size() != waypoints.size()) {
    return true;
  }

  for (size_t i = 0; i < last_waypoints_->size(); ++i) {
    if (!arePosesWithinThreshold(last_waypoints_.value()[i], waypoints[i])) {
      return true;
    }
  }

  // No need to send a new plan message
  return false;
}

bool Plan2Generator::arePosesWithinThreshold(const Pose2d& pose1, const Pose2d& pose2) {
  const Vector2d threshold = get_new_message_threshold();
  const Vector2d delta = PoseMagnitude(pose1.inverse() * pose2);
  return (delta[0] < threshold[0] && delta[1] < threshold[1]);
}

}  // namespace message_generators
}  // namespace isaac
