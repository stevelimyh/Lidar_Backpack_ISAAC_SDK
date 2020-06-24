/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "LatticeGenerator.hpp"

#include <string>

#include "messages/math.hpp"

namespace isaac {
namespace message_generators {

void LatticeGenerator::start() {
  tickPeriodically();
}

void LatticeGenerator::tick() {
  // Set the frame name of the lattice and reference frame for the pose
  const std::string lattice_frame_name = get_lattice_frame_name();
  const std::string reference_frame_name = get_reference_frame_name();
  reference_frame_T_lattice_.setLhsName(reference_frame_name);
  reference_frame_T_lattice_.setRhsName(lattice_frame_name);
  // Read the cell size, dimensions of the lattice and the relative offset of the reference frame
  // with respect to the lattice. These are required to compute the pose of the reference frame
  // relative to the lattice.
  const double cell_size = get_cell_size();
  const Vector2i lattice_dimensions = get_dimensions();
  const Vector2d relative_offset = get_relative_offset();
  // Compute the relative pose of the reference frame with respect to the lattice
  const Vector2d lattice_offset =
      relative_offset.array() * lattice_dimensions.array().cast<double>();
  const Pose2d reference_frame_T_lattice = Pose2d::Translation(cell_size * lattice_offset);
  set_reference_frame_T_lattice(reference_frame_T_lattice, getTickTime());
  // Create the lattice proto message to publish
  auto gridmap_lattice_proto = tx_gridmap_lattice().initProto();
  gridmap_lattice_proto.setCellSize(cell_size);
  gridmap_lattice_proto.setFrame(lattice_frame_name);
  ToProto(lattice_dimensions, gridmap_lattice_proto.initDimensions());
  tx_gridmap_lattice().publish();
}

}  // namespace message_generators
}  // namespace isaac
