/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <deque>
#include <memory>
#include <shared_mutex>  // NOLINT
#include <string>
#include <unordered_map>

#include "engine/alice/component.hpp"
#include "engine/gems/cask/cask.hpp"

namespace isaac {
namespace composite {

// A database which provides access to composite protos as waypoints. The waypoints are loaded from
// a cask file on a lazy-loading basics, and stored as CompositeWaypoint.
class CompositeAtlas : public alice::Component {
 public:
  // Represents a waypoint from composite proto by a schema specifying the quantities at this
  // waypoint, and a vector with values of quantities as the state at this waypoint.
  struct CompositeWaypoint {
    // Schema in composite proto. This should be used to parser incoming proto message for
    // calculating distance to the current waypoint.
    composite::Schema schema;
    // State of the waypoint loaded from values in composite proto using the above schema.
    VectorXd values;
  };

  // release cask
  void deinitialize() override;

  // Returns a CompositeWaypoint for the given name as uuid in cask. Returns nullptr if it does not
  // exist, or unable to parse either schema or values from the proto loaded from cask.
  const CompositeWaypoint* getWaypoint(const std::string& name);

  // Cask filename which contains composite protos.
  ISAAC_PARAM(std::string, cask);

 private:
  // Loads a give waypoint from cask and adds it to the waypoints queue and map.
  const CompositeWaypoint* loadWaypoint(const std::string& name);
  // Cask object to read waypoints from.
  std::unique_ptr<cask::Cask> cask_;
  // Cached all waypoints read from cask.
  std::deque<CompositeWaypoint> waypoints_;
  // Map of name to waypoints.
  std::unordered_map<std::string, const CompositeWaypoint*> waypoints_map_;
  // A multi-read single-write mutex for accessing cask and waypoints.
  std::shared_timed_mutex mutex_;
};

}  // namespace composite
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::composite::CompositeAtlas)
