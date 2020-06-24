/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "MocapTest.hpp"

#include <vector>

#include "engine/core/math/pose3.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"
#include "messages/math.hpp"
#include "messages/pose_tree.capnp.h"

namespace isaac {
namespace vicon {

  void MocapTest::start() {
    tickOnMessage(rx_vicon_pose_tree());
  }

  // Display the pose tree's size and plot markers at every frame of motion
  // capture
  void MocapTest::tick() {
    int64_t acqtime = rx_vicon_pose_tree().acqtime();
    show("acqtime", acqtime);

    auto message_vicon_pose_tree = rx_vicon_pose_tree().getProto();
    auto message_vicon_markers = rx_vicon_markers().getProto();

    capnp::List<PoseTreeProto::Node>::Reader nodes =
        message_vicon_pose_tree.getNodes();
    const unsigned int num_nodes = nodes.size();
    show("num_nodes", num_nodes);

    capnp::List<PoseTreeProto::Edge>::Reader edges =
        message_vicon_pose_tree.getEdges();
    const unsigned int num_edges = edges.size();
    show("num_edges", num_edges);

    capnp::List<MarkerListProto::Marker>::Reader markers =
        message_vicon_markers.getMarkers();
    const unsigned int num_markers = markers.size();

    std::vector<Vector3d> vis_markers;
    show("markers", [&](sight::Sop& sop) {
      vis_markers.reserve(num_markers + 1);
      vis_markers.clear();
      sop.transform = sight::SopTransform(isaac::Pose3d::Identity());
      sop.style = sight::SopStyle{"cyan"};
      for (unsigned int i = 0; i < num_markers; ++i) {
        // plot all the markers, converting millimeters to decimeters to
        // visualize better
        Vector3d world_T_marker =
            FromProto(markers[i].getWorldTMarker()) / 100.0;
        world_T_marker.z() =
            -1 * world_T_marker.z();  // switch the Z sign for web sight
        vis_markers.push_back(world_T_marker);
        sop.add(world_T_marker);
      }

      if (get_draw_line_segments()) {
        vis_markers.push_back(vis_markers[0]);
        sop.add(vis_markers);
      }
    });
  }

  void MocapTest::stop() {}

}  // namespace vicon
}  // namespace isaac
