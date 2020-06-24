/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <mutex>
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "packages/map/Layer.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace map {

// A map layer which holds annotated waypoints and provides various methods to access them
class WaypointMapLayer : public Layer {
 public:
  // A waypoint stored in the map
  struct Waypoint {
    // The pose of the waypoint in the reference coordinate frame
    Pose3d world_T_waypoint;
    // The radius of the waypoint
    double radius;
    // A color used for visualization
    Vector3ub color;
  };

  // A set of named waypoints
  using NamedWaypoints = std::map<std::string, Waypoint>;

  void start() override;
  Json toJson() const override;

  // Returns true if a waypoint is valid
  bool hasWaypoint(const std::string& name) const { return findByName(name) != std::nullopt; }
  // Finds a named waypoint
  std::optional<Waypoint> findByName(const std::string& name) const;

  // Add or update existing layer from JSON
  void addOrUpdateFromJson(const nlohmann::json& json);

  // Deletes a waypoint
  void deleteWaypoint(const std::string& name);

  // A json object from configuration containing the waypoints.
  //
  // .. code-block:: javascript
  //
  //    Layout:
  //      {
  //        "wp1": { "pose": [1,0,0,0,0,0,0], "radius": 0.5 },
  //        "wp3": { "pose": [1,0,0,0,0.1,-1.2,0], "color": [54.0, 127.0, 255.0] }
  //      }
  ISAAC_PARAM(nlohmann::json, waypoints, nlohmann::json::object())

 private:
  // Updates JSON and visualization based on stored waypoints
  void syncWaypoints();

  mutable std::mutex mutex_;
  NamedWaypoints waypoints_;
};

}  // namespace map
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::map::WaypointMapLayer);
