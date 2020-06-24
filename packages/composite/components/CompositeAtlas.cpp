/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CompositeAtlas.hpp"

#include <deque>
#include <memory>
#include <shared_mutex>  // NOLINT
#include <string>
#include <utility>

#include "engine/alice/node.hpp"
#include "engine/alice/utils/utils.hpp"
#include "engine/gems/cask/cask.hpp"
#include "packages/composite/gems/parser.hpp"

namespace isaac {
namespace composite {

void CompositeAtlas::deinitialize() {
  cask_.reset();
}

const CompositeAtlas::CompositeWaypoint* CompositeAtlas::getWaypoint(const std::string& name) {
  // multiple read threads allowed
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    const auto it = waypoints_map_.find(name);
    if (it != waypoints_map_.end()) {
      // waypoint is already loaded, simply return
      return it->second;
    }
  }
  // waypoint is not loaded, load it from cask first. only a single write thread allowed
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  return loadWaypoint(name);
}

const CompositeAtlas::CompositeWaypoint* CompositeAtlas::loadWaypoint(const std::string& name) {
  // check again in case this is loaded by another thread before the write lock is acquired
  const auto it = waypoints_map_.find(name);
  if (it != waypoints_map_.end()) {
    return it->second;
  }

  // lazy loading
  if (!cask_) {
    cask_ = std::make_unique<cask::Cask>(get_cask(), cask::Cask::Mode::Read);
  }

  // read from cask
  const auto message = alice::ReadMessageFromCask(Uuid::FromString(name), *cask_);
  auto* proto_message = dynamic_cast<const alice::ProtoMessageBase*>(message.get());
  if (proto_message == nullptr) {
    LOG_WARNING("Cannot get message from cask");
    return nullptr;
  }
  auto reader = proto_message->reader().getRoot<CompositeProto>();

  // get schema
  const auto maybe_schema = composite::ReadSchema(reader);
  if (!maybe_schema) {
    LOG_WARNING("Cannot get schema from message");
    waypoints_map_.insert({name, nullptr});
    return nullptr;
  }
  // get values
  composite::Parser parser;
  parser.requestSchema(*maybe_schema);
  VectorXd values(maybe_schema->getElementCount());
  if (!parser.parse(reader, message->buffers, values)) {
    LOG_WARNING("Cannot read values for waypoint");
    waypoints_map_.insert({name, nullptr});
    return nullptr;
  }

  // add to queue and map
  waypoints_.push_back({std::move(*maybe_schema), std::move(values)});
  const CompositeWaypoint* waypoint = &waypoints_.back();
  waypoints_map_.insert({name, waypoint});
  return waypoint;
}

}  // namespace composite
}  // namespace isaac
