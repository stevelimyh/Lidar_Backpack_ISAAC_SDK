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

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/kinematic_tree/kinematic_tree.hpp"
#include "messages/composite.capnp.h"

namespace isaac {
namespace map {

// Loads a kinematic tree from file and provides access to the model
class KinematicTree : public alice::Codelet {
 public:
  void start() override;

  // Path to a file to load the kinematic tree from
  ISAAC_PARAM(std::string, kinematic_file);

  // Returns a constant reference to the kinematic tree. This can only be used after the model is
  // loaded from file in start().
  const kinematic_tree::KinematicTree& model() const;

 private:
  // Tries to load a kinematic tree from file. return true if success
  bool loadFromFile(const std::string& filename);

  // Kinematic tree object loaded from file
  std::optional<kinematic_tree::KinematicTree> kinematic_tree_;
};

}  // namespace map
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::map::KinematicTree);
