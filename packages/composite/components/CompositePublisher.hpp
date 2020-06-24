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
#include "engine/core/optional.hpp"
#include "messages/composite.capnp.h"
#include "packages/composite/components/CompositeAtlas.hpp"
#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace composite {

// Publishes a composite proto containing a batch of waypoints specify on a path. The waypoints
// are read from a CompositeAtlas database.
class CompositePublisher : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Publishes all the waypoints in path as a batch. A new message is published only when the path
  // changes.
  ISAAC_PROTO_TX(CompositeProto, path);

  // Name of the node containing the CompositeAtlas component.
  ISAAC_PARAM(std::string, atlas);
  // The list of waypoint names as the path to follow. Reports failure if any waypoint on the
  // path does not exist in CompositeAtlas. A new message is sent if this config changes.
  ISAAC_PARAM(std::vector<std::string>, path, {});
  // If true, use the schema set in config below. Otherwise use schema of the first waypoint in the
  // path read from cask as the schema for the whole path, and ignore the schema set in config.
  ISAAC_PARAM(bool, use_config_schema, false);
  // Sets the schema to publish. This must be a subset of schemas in the path waypoints. Only
  // used if use_waypoint_schema is set to false.
  ISAAC_PARAM(Schema, schema);
  // If true, report success after publishing a valid path.
  ISAAC_PARAM(bool, report_success, false);

 private:
  // Pointer to the CompositeAtlas component
  CompositeAtlas* atlas_;
  // Waypoints sent in the last message
  std::optional<std::vector<std::string>> maybe_previous_waypoints_;
  // Schema used to publish the path.
  std::optional<composite::Schema> schema_;
};

}  // namespace composite
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::composite::CompositePublisher);
