/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

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
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "messages/composite.capnp.h"
#include "packages/composite/components/CompositeMetric.hpp"
#include "packages/composite/gems/fragment_index.hpp"
#include "packages/composite/gems/parser.hpp"
#include "packages/composite/gems/serializer.hpp"

namespace isaac {
namespace composite {

// Receives a sequence of waypoints via a message and publishes the waypoint one by one. The next
// waypoint is published when the current state is within tolerance of the current waypoint. When
// a new path is received, the current waypoint is reset to the first waypoint in the path. This
// codelet requires a CompositeMetric component in the same node to specify how to compute
// distance between the waypoint and the received state message.
class FollowPath : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Receives the path to follow given as a batch of waypoints
  ISAAC_PROTO_RX(CompositeProto, path);
  // Receives current state, used to check if we arrive at the current waypoint.
  ISAAC_PROTO_RX(CompositeProto, state);

  // Publishes the desired goal waypoint.
  ISAAC_PROTO_TX(CompositeProto, goal);

  // Seconds to wait after arriving at the current waypoint before publishing the next waypoint.
  ISAAC_PARAM(double, wait_time, 1.0);
  // If set to true we will repeat the path once completed. Otherwise will report success on
  // completion.
  ISAAC_PARAM(bool, loop, false);
  // Tolerance for arrival. A waypoint is reached when the distance between state and current
  // waypoint computed by the CompositeMetric component is below this limit.
  ISAAC_PARAM(double, tolerance, 0.1);

 private:
  // Sends a new waypoint goal message
  void sendNewWaypoint();

  // The current waypoint index in the path
  int waypoint_index_;
  // Stores the waypoint values received from the latest path message as rows in the matrix.
  MatrixXd waypoints_;
  // Timestamp of the current goal message. Only use state newer than this time to determine arrival
  int64_t path_acqtime_;
  // To determine whether robot waited long enough upon arrival
  std::optional<double> has_arrived_time_;

  // Pointer to CompositeMetric component in this node
  CompositeMetric* metric_;
  // Schema used by the CompositeMetric component
  Schema metric_schema_;
  // Parser with metric schema as the requested schema, to parser state for CompositeMetric
  // distance input vector
  Parser metric_parser_;
  // Schema of the latest path message. This is the schema for all the waypoints.
  Schema waypoint_schema_;
  // A indexer to read the values requested by the metric schema from the waypoint schema.
  FragmentIndex fragment_index_;
  // The current Waypoint values used to compute distance. Read from the waypoint using fragment
  // index above.
  std::optional<VectorXd> waypoint_values_;
};

}  // namespace composite
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::composite::FollowPath);
