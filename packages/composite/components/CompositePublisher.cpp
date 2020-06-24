/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CompositePublisher.hpp"

#include <string>
#include <utility>
#include <vector>

#include "engine/core/optional.hpp"
#include "messages/tensor.hpp"
#include "packages/composite/components/CompositeAtlas.hpp"
#include "packages/composite/gems/fragment_index.hpp"
#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace composite {

void CompositePublisher::start() {
  atlas_ = node()->app()->findComponentByName<CompositeAtlas>(get_atlas());
  if (atlas_ == nullptr) {
    reportFailure("Could not find composite atlas '%s'", get_atlas());
    return;
  }
  maybe_previous_waypoints_ = std::nullopt;
  tickPeriodically();
}

void CompositePublisher::tick() {
  // Do not send a new message if waypoints not available or has not changed
  const std::vector <std::string> current_waypoints = get_path();
  if (maybe_previous_waypoints_ && *maybe_previous_waypoints_ == current_waypoints) {
    return;
  }
  maybe_previous_waypoints_ = current_waypoints;
  int batch = maybe_previous_waypoints_->size();
  if (batch == 0) {
    return;
  }

  // get schema from config or first waypoint
  if (get_use_config_schema()) {
    schema_ = try_get_schema();
  } else {
    const std::string name = (*maybe_previous_waypoints_)[0];
    const CompositeAtlas::CompositeWaypoint* waypoint = atlas_->getWaypoint(name);
    if (!waypoint) {
      reportFailure("Waypoint %s not in atlas.", name.c_str());
      return;
    }
    std::vector<Quantity> quantities = waypoint->schema.getQuantities();
    schema_ = Schema::Create(std::move(quantities), waypoint->schema.getHash());
  }
  if (!schema_) {
    reportFailure("Cannot read schema from config");
    return;
  }
  // get values for all waypoints on path
  Tensor3d values(batch, 1, schema_->getElementCount());
  for (int i = 0; i < batch; i++) {
    const std::string name = (*maybe_previous_waypoints_)[i];
    const CompositeAtlas::CompositeWaypoint* waypoint = atlas_->getWaypoint(name);
    if (!waypoint) {
      reportFailure("Waypoint %s not in atlas.", name.c_str());
      return;
    }
    // parse value from waypoint
    const auto maybe_fragment_index =
        composite::FragmentIndex::Create(waypoint->schema, *schema_, *schema_);
    // fail in case of incompatible schema
    if (!maybe_fragment_index) {
      reportFailure("Publisher schema incompatible with waypoint %s schema.", name.c_str());
      return;
    }
    // copy waypoint values to output tensor
    auto dest = values.matrix(i, 0);
    maybe_fragment_index->copyFragments(waypoint->values.data(), waypoint->values.size(),
                                        dest.data(), dest.size());
  }

  // publish message
  auto proto_builder = tx_path().initProto();
  composite::WriteSchema(*schema_, proto_builder);
  ToProto(std::move(values), proto_builder.initValues(), tx_path().buffers());
  tx_path().publish();

  // report success after publish a path
  if (get_report_success()) {
    reportSuccess();
  }
}

}  // namespace composite
}  // namespace isaac
