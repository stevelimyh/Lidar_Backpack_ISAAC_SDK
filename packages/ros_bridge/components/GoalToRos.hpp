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

#include "messages/differential_base.capnp.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "packages/ros_bridge/components/ProtoToRosConverter.hpp"

namespace isaac {
namespace ros_bridge {

// This codelet receives goal as message within Isaac application and publishes it to ROS as a
// message. If goal feedback is needed, use similar codelet named "GoalToRosAction" instead.
class GoalToRos : public ProtoToRosConverter<Goal2Proto, move_base_msgs::MoveBaseActionGoal, true> {
 public:
  bool protoToRos(Goal2Proto::Reader reader, const ros::Time& ros_time,
                  move_base_msgs::MoveBaseActionGoal& ros_message) override;

  // Frame of the goal in outgoing message
  ISAAC_PARAM(std::string, goal_frame, "map");
  // Frame of the robot in ROS. Used to stop the robot if needed.
  ISAAC_PARAM(std::string, robot_frame, "base_link");
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::GoalToRos);
