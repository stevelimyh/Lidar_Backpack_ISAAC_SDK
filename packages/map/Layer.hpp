/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace map {

// Base class for map layer of a map node.
class Layer : public alice::Codelet {
 public:
  virtual ~Layer() = default;

  // Serializes the map layer to JSON
  virtual Json toJson() const = 0;

  // Returns the current version of this layer
  int64_t version() const {
    return version_;
  }

  // Gets the name of the map frame
  std::string getMapFrameName() const;

 protected:
  // Contain the version of this layer. It can be used to easy detect changes and validity.
  int64_t version_ = 0;
};

}  // namespace map
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::map::Layer);
