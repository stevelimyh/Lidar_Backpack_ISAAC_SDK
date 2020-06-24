/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "FollowPath.hpp"

#include <string>
#include <utility>

#include "engine/core/math/types.hpp"
#include "messages/tensor.hpp"
#include "packages/composite/components/CompositeMetric.hpp"
#include "packages/composite/gems/fragment_index.hpp"
#include "packages/composite/gems/parser.hpp"
#include "packages/composite/gems/serializer.hpp"

namespace isaac {
namespace composite {

void FollowPath::start() {
  metric_ = node()->getComponent<CompositeMetric>();
  path_acqtime_ = 0;
  has_arrived_time_ = std::nullopt;
  tickPeriodically();
}

void FollowPath::tick() {
  if (rx_path().available() && rx_path().acqtime() > path_acqtime_) {
    // new path is available. get waypoint schema
    const auto maybe_waypoint_schema = ReadSchema(rx_path().getProto());
    if (!maybe_waypoint_schema) {
      reportFailure("Cannot get schema from path message.");
    }
    waypoint_schema_ = *maybe_waypoint_schema;
    Parser parser;
    parser.requestSchema(waypoint_schema_);
    if (!parser.parse(rx_path().getProto(), rx_path().buffers(), waypoints_)) {
      reportFailure("Cannot get waypoint values from path message.");
      return;
    }
    // try set and then get metric schema
    metric_->setOrLoadSchema(&waypoint_schema_);
    const std::optional<Schema> maybe_schema_ = metric_->getSchema();
    if (!maybe_schema_) {
      reportFailure("Cannot get schema from CompositeMetric for distance calculation.");
      return;
    }
    metric_schema_ = *maybe_schema_;
    metric_parser_.requestSchema(metric_schema_);
    // get fragment_index to parse values in metric schema from waypoint schema
    const auto maybe_fragment_index =
        FragmentIndex::Create(waypoint_schema_, metric_schema_, metric_schema_);
    if (!maybe_fragment_index) {
      reportFailure("Cannot get schema for metric from waypoint schema.");
      return;
    }
    fragment_index_ = *maybe_fragment_index;
    // publish first waypoint on new path
    path_acqtime_ = rx_path().acqtime();
    // TODO(qianl): add option to start from closet waypoint
    waypoint_index_ = 0;
    sendNewWaypoint();
  }

  if (has_arrived_time_ && (this->getTickTime() - (*has_arrived_time_)) > get_wait_time()) {
    // arrival criteria met. move to next waypoint on path or report success
    waypoint_index_++;
    if (waypoint_index_ >= waypoints_.cols()) {
      if (get_loop()) {
        waypoint_index_ = 0;
      } else {
        reportSuccess();
        return;
      }
    }
    sendNewWaypoint();
  }

  // read current state and check distance to goal if the waypoint is known
  if (waypoint_values_ && rx_state().available()) {
    VectorXd state(metric_schema_.getElementCount());
    if (!metric_parser_.parse(rx_state().getProto(), rx_state().buffers(), state)) {
      reportFailure("Unable to parser states required for CompositeMetric from message.");
      return;
    }
    std::optional<double> delta = metric_->distance(*waypoint_values_, state);
    if (!delta) {
      reportFailure("Cannot calculate distance between state and waypoint with metric.");
      return;
    }
    show("distance", *delta);
    if (*delta < get_tolerance() && !has_arrived_time_) {
      has_arrived_time_ = getTickTime();
    }
  }
}

void FollowPath::sendNewWaypoint() {
  has_arrived_time_ = std::nullopt;
  LOG_INFO("Send new target waypoint %d of %d.", waypoint_index_, waypoints_.cols());
  VectorXd values = waypoints_.col(waypoint_index_);
  waypoint_values_ = VectorXd(metric_schema_.getElementCount());
  fragment_index_.copyFragments(values.data(), values.size(), waypoint_values_->data(),
                                waypoint_values_->size());
  // Publish the new waypoint
  auto proto_builder = tx_goal().initProto();
  composite::WriteSchema(waypoint_schema_, proto_builder);
  composite::Serializer serializer;
  serializer.setSchema(waypoint_schema_);
  if (!serializer.serialize(std::move(values), proto_builder, tx_goal().buffers())) {
    reportFailure("Could not serialize waypoint.");
    return;
  }
  tx_goal().publish();
}

}  // namespace composite
}  // namespace isaac
