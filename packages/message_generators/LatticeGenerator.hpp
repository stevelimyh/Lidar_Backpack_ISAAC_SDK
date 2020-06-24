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

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/types.hpp"
#include "messages/map.capnp.h"

namespace isaac {
namespace message_generators {

// Creates a lattice proto which represents a grid map. This information includes the cell size,
// name of the lattice frame and dimensions of the lattice in pixels. The codelet also computes the
// pose of the reference frame with respect to the lattice using the dimensions of the lattice and
// the relative offset of the reference frame with respect to the lattice. This pose is set in the
// pose tree.
class LatticeGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output lattice proto. This contains relevant information about the corresponding gridmap.
  ISAAC_PROTO_TX(LatticeProto, gridmap_lattice);

  // Parameter defining the cell size in metres.
  ISAAC_PARAM(double, cell_size, 0.05);
  // The dimensions of the grid map in pixels
  ISAAC_PARAM(Vector2i, dimensions, Vector2i(256, 256));
  // The name of the lattice coordinate frame. This will be used to write the pose of the gridmap
  // relative to the reference frame in the pose tree.
  ISAAC_PARAM(std::string, lattice_frame_name, "gridmap_frame");
  // Name of the reference frame
  ISAAC_PARAM(std::string, reference_frame_name, "ref");
  // Percentage offset of robot relative to the map. The offset determines the position of the
  // robot (or the reference frame) with respect to the grid map created.
  // The origin of the grid map is considered to be at the top-left of the grid.
  // The x parameter defines the percentage offset for the rows (positive is in the upward
  // direction and negative is in the downward direction), and the y parameter defines the offset
  // for the columns (positive is in the left direction and negative is in the right direction).
  // Determining the offset using a percentage basis makes it agnostic to the dimensions of the map.
  // The default value fixes the reference frame at the top-center of the grid map.
  ISAAC_PARAM(Vector2d, relative_offset, Vector2d(0.0, -0.5));

  // Relative pose of the map with respect to the reference frame
  ISAAC_POSE2(reference_frame, lattice);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::LatticeGenerator);
