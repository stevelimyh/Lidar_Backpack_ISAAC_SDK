/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "messages/differential_base.capnp.h"
#include "packages/ros_bridge/gems/include_before_ros.hpp"

// Include ROS headers after include_before_ros.hpp. See include_before_ros.hpp for details.
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"

namespace isaac {
namespace ros_bridge {

// This codelet receives goal as message within Isaac application and publishes it to ROS as an
// action. Unlike the similar codelet named "GoalToRos", GoalToRosAction then publishes
// Goal2FeedbackProto.
class GoalToRosAction : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // The target destination
  ISAAC_PROTO_RX(Goal2Proto, goal);
  // The odometry information with current speed
  ISAAC_PROTO_RX(Odometry2Proto, odometry);

  // Feedback regarding the goal
  ISAAC_PROTO_TX(Goal2FeedbackProto, feedback);

  // ROS namespace where action will be communicated to
  ISAAC_PARAM(std::string, action_name, "move_base");
  // Frame of the goal in outgoing ROS message
  ISAAC_PARAM(std::string, goal_frame_ros, "map");
  // Frame of the robot in ROS. Used to stop the robot if needed.
  ISAAC_PARAM(std::string, robot_frame_ros, "base_link");
  // Frame of the robot in Isaac. Used in publishing feedback pose.
  ISAAC_PARAM(std::string, robot_frame_isaac, "robot");
  // Threshold on speed to determine if the robot is stationary (positional and angular)
  ISAAC_PARAM(Vector2d, stationary_speed_thresholds, Vector2d(0.025, DegToRad(5.0)));

 private:
  // Information about the desired goal
  struct Goal {
    // Acquisition time to which the goal relates
    int64_t acqtime;
    // The name of the goal coordinate frame
    std::string frame_name;
    // The pose of the goal in the goal coordinate frame
    Pose2d frame_T_goal;
  };

  // Sends the goal as action if any goal message is received
  void convertAndSendGoal();
  // Publishes feedback regarding the last goal sent to ROS
  void publishFeedback();
  // Returns true if the robot is considered to be stationary
  bool isStationary();

  // Last goal sent to ROS
  std::optional<Goal> last_goal_ = std::nullopt;
  // Action client to communicate with ROS
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client_;
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::GoalToRosAction);
