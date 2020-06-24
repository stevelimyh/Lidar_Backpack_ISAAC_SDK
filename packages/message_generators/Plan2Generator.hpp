/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/differential_base.capnp.h"

namespace isaac {
namespace message_generators {

// Publishes a plan which is populated with the waypoints specified via parameters. Waypoints can be
// either listed as poses directly, or as names of the frames to look-up from the PoseTree.
class Plan2Generator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // The plan generated as specified via parameters
  ISAAC_PROTO_TX(Plan2Proto, plan);

  // List of waypoint poses in the form of (angle, x, y).
  // Example configuration:
  // "waypoints": [
  //   [4.15, 26.1, 9.58],
  //   [1.578, 26.14, 14.75]
  // ]
  // Either 'waypoints' or 'frames' parameter needs to be set.
  ISAAC_PARAM(std::vector<Pose2d>, waypoints);
  // List of waypoints as frame names defined in PoseTree.
  // Either 'waypoints' or 'frames' parameter needs to be set.
  ISAAC_PARAM(std::vector<std::string>, frames);
  // Frame for the waypoints. Sets the plan frame in outgoing message.
  ISAAC_PARAM(std::string, plan_frame, "world");
  // Name of a frame that is not moving. Used to decide whether a new plan message needs to be
  // published.
  ISAAC_PARAM(std::string, static_frame, "world");
  // A new message will be published whenever change in poses exceeds this threshold.
  // Values are for Euclidean distance and angle respectively.
  ISAAC_PARAM(Vector2d, new_message_threshold, Vector2d(1e-3, DegToRad(0.01)));

 private:
  // Returns true if difference between the two poses is less than new_message_threshold.
  bool arePosesWithinThreshold(const Pose2d& pose1, const Pose2d& pose2);
  // Returns true if the plan is different from the previous one.
  bool isPlanNew(const std::string& plan_frame, const Pose2d& static_T_plan,
                 const std::vector<Pose2d>& waypoints);

  // Store information regarding the last plan sent as message. This is information is used not to
  // send duplicate plan messages.
  std::optional<std::string> last_frame_;
  std::optional<std::vector<Pose2d>> last_waypoints_;
  std::optional<Pose2d> last_static_T_plan_;
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::Plan2Generator);
