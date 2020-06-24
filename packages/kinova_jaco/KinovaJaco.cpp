/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "KinovaJaco.hpp"

#include <string>
#include <utility>
#include <vector>

#include "KinovaTypes.h"
#include "engine/core/math/types.hpp"
#include "packages/composite/gems/measure.hpp"
#include "packages/composite/gems/parser.hpp"
#include "packages/map/KinematicTree.hpp"

namespace isaac {
namespace kinova_jaco {

namespace {
constexpr int kNumJoints = 7;  // number of joints
}  // namespace

void KinovaJaco::start() {
  // Load Jaco SDK and initialize USB connection with the arm
  successful_api_open_ = kinova_jaco_api_.open(get_kinova_jaco_sdk_path());
  if (!successful_api_open_) {
    reportFailure("Fails to open Kinova API.");
    return;
  }

  // Try to get the map::KinematicTree component
  const auto maybe_kinematic_tree_node = try_get_kinematic_tree();
  if (!maybe_kinematic_tree_node) {
    reportFailure("KinematicTree node is not specified.");
    return;
  }
  const auto maybe_kinematic_tree_component =
      node()->app()->getNodeComponentOrNull<map::KinematicTree>(*maybe_kinematic_tree_node);
  if (!maybe_kinematic_tree_component) {
    reportFailure("Node %s does not contain KinematicTree component.",
                  maybe_kinematic_tree_node->c_str());
    return;
  }
  // Get the kinematic_tree object and validate against hardware
  const kinematic_tree::KinematicTree& model = maybe_kinematic_tree_component->model();
  if (!validateKinematicTree(model)) {
    reportFailure("Kinematic tree model does not match hardware.");
    return;
  }

  // Initialize schema to publish state
  initStateSchema();

  // Initialize arm and finger to home position. These API calls are blocking.
  if (get_initialize_home()) {
    // Move joints to home position
    kinova_jaco_api_.moveHome();
    // Move finger to fully outstretched, ready to grasp
    kinova_jaco_api_.initFingers();
  }

  tickPeriodically();
}

void KinovaJaco::tick() {
  // Send a cartesian position command if there is a new cartesian pose command message
  if (rx_arm_command().available()) {
    const int64_t time = rx_arm_command().acqtime();
    if (!last_command_time_ || time > *last_command_time_) {
      // If message is the first received message (last_command_time_ not initialized) or a more
      // recent command, parse and send to arm.
      last_command_time_ = time;
      parseCommand();
    }
  }

  // Publish current state of the arm
  publishState();
}

void KinovaJaco::stop() {
  // Close connection with Jaco arm
  if (successful_api_open_) {
    // Remove all remaining points in the FIFO
    kinova_jaco_api_.eraseAllTrajectories();
    kinova_jaco_api_.close();
  }
}

bool KinovaJaco::validateKinematicTree(const kinematic_tree::KinematicTree& model) {
  // Number of active links match number of joints
  const auto& links = model.getActiveLinks();
  if (links.size() != kNumJoints) {
    return false;
  }
  // Each active link only has one degree of freedom
  if (model.getMotorStateDimensions() != kNumJoints) {
    return false;
  }
  // Kinematic tree is valid, get the list of joint names for parsing composite message
  joint_names_.clear();
  for (const auto* link : links) {
    joint_names_.push_back(link->name);
  }
  return true;
}

void KinovaJaco::initCommandParser(const ControlMode mode) {
  const std::string end_effector_name = get_end_effector_name();
  switch (mode) {
    case ControlMode::kEndEffectorPose:
      command_parser_.requestSchema(composite::Schema(
          {{end_effector_name, composite::Measure::kPosition, VectorXi::Constant(1, 3)},
           {end_effector_name, composite::Measure::kRotation, VectorXi::Constant(1, 4)}}));
      break;
    case ControlMode::kEndEffectorVelocity:
      command_parser_.requestSchema(composite::Schema(
          {{end_effector_name, composite::Measure::kSpeed, VectorXi::Constant(1, 3)},
           {end_effector_name, composite::Measure::kAngularSpeed, VectorXi::Constant(1, 3)}}));
      break;
    case ControlMode::kJointPosition:
      command_parser_.requestSchema(composite::Schema(joint_names_, composite::Measure::kPosition));
      break;
    case ControlMode::kJointVelocity:
      command_parser_.requestSchema(composite::Schema(joint_names_, composite::Measure::kSpeed));
      break;
    default:
      command_parser_.requestSchema({});
      return;
  }
}

void KinovaJaco::initStateSchema() {
  std::vector<composite::Quantity> quantities;
  // Add end effector pose
  const std::string end_effector_name = get_end_effector_name();
  quantities.push_back(
      composite::Quantity::Vector(end_effector_name, composite::Measure::kPosition, 3));
  quantities.push_back(
      composite::Quantity::Vector(end_effector_name, composite::Measure::kRotation, 4));
  // Add joint position and speed
  for (int i = 0; i < kNumJoints; i++) {
    quantities.push_back(
        composite::Quantity::Scalar(joint_names_[i], composite::Measure::kPosition));
  }
  for (int i = 0; i < kNumJoints; i++) {
    quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kSpeed));
  }
  state_schema_ = composite::Schema(std::move(quantities));
}

void KinovaJaco::parseCommand() {
  const ControlMode mode = get_control_mode();
  if (mode == ControlMode::kInvalid) {
    reportFailure("Invalid control mode");
    return;
  }
  // Reset command parser if control mode changes in config
  if (control_mode_ != mode) {
    initCommandParser(mode);
    control_mode_ = mode;
  }

  TrajectoryPoint trajectory_point;
  trajectory_point.InitStruct();
  trajectory_point.LimitationsActive = 0;
  trajectory_point.Position.HandMode = HAND_NOMOVEMENT;

  if (mode == ControlMode::kJointPosition || mode == ControlMode::kJointVelocity) {
    // Joint control mode
    VectorXd command(kNumJoints);
    if (!command_parser_.parse(rx_arm_command().getProto(), rx_arm_command().buffers(),
                               command)) {
      reportFailure("Fails to parse joint command");
      return;
    }
    if (mode == ControlMode::kJointVelocity) {
      trajectory_point.Position.Type = ANGULAR_VELOCITY;
    } else {
      trajectory_point.Position.Type = ANGULAR_POSITION;
      // Kinova stores commanded position in FIFO. Need to clear trajectory to accept new joint
      // angle as target
      kinova_jaco_api_.eraseAllTrajectories();
    }
    auto& actuators = trajectory_point.Position.Actuators;
    // Kinova API requires joint angle in degree and joint speed in degree/s
    actuators.Actuator1 = static_cast<float>(RadToDeg(command(0)));
    actuators.Actuator2 = static_cast<float>(RadToDeg(command(1)));
    actuators.Actuator3 = static_cast<float>(RadToDeg(command(2)));
    actuators.Actuator4 = static_cast<float>(RadToDeg(command(3)));
    actuators.Actuator5 = static_cast<float>(RadToDeg(command(4)));
    actuators.Actuator6 = static_cast<float>(RadToDeg(command(5)));
    actuators.Actuator7 = static_cast<float>(RadToDeg(command(6)));
    for (int i = 0; i < 7; i++) {
      show("actuator" + std::to_string(i + 1) + ".command", command(i));
    }
  } else {
    // End effector control mode
    CartesianInfo& cartesian_info = trajectory_point.Position.CartesianPosition;
    if (mode == ControlMode::kEndEffectorPose) {
      // Set end effector pose command
      Vector7d command;
      trajectory_point.Position.Type = CARTESIAN_POSITION;
      if (!command_parser_.parse(rx_arm_command().getProto(), rx_arm_command().buffers(),
                                 command)) {
        reportFailure("Fails to parse end effector pose command");
        return;
      }
      // Kinova stores commanded position in FIFO. Need to clear trajectory to accept new end
      // effector pose as target
      kinova_jaco_api_.eraseAllTrajectories();
      cartesian_info.X = static_cast<float>(command(0));
      cartesian_info.Y = static_cast<float>(command(1));
      cartesian_info.Z = static_cast<float>(command(2));
      // Convert from quaternion to euler angles (roll/X, pitch/Y, yaw/Z)
      // Kinova API requires cartesian angle in rad and cartesian angular velocity in rad/s
      auto orientation_SO3 = SO3d::FromQuaternion(Quaterniond(command.tail<4>()));
      auto orientation_euler_angles = orientation_SO3.eulerAnglesRPY();
      cartesian_info.ThetaX = static_cast<float>(orientation_euler_angles[0]);
      cartesian_info.ThetaY = static_cast<float>(orientation_euler_angles[1]);
      cartesian_info.ThetaZ = static_cast<float>(orientation_euler_angles[2]);
    } else {
      // Set end effector velocity command
      Vector6d command;
      trajectory_point.Position.Type = CARTESIAN_VELOCITY;
      if (!command_parser_.parse(rx_arm_command().getProto(), rx_arm_command().buffers(),
                                 command)) {
        reportFailure("Fails to parse end effector velocity command");
        return;
      }
      cartesian_info.X = static_cast<float>(command(0));
      cartesian_info.Y = static_cast<float>(command(1));
      cartesian_info.Z = static_cast<float>(command(2));
      cartesian_info.ThetaX = static_cast<float>(command(3));
      cartesian_info.ThetaY = static_cast<float>(command(4));
      cartesian_info.ThetaZ = static_cast<float>(command(5));
    }
    show("endeffectorX.command", cartesian_info.X);
    show("endeffectorY.command", cartesian_info.Y);
    show("endeffectorZ.command", cartesian_info.Z);
    show("endeffectorThetaX.command", cartesian_info.ThetaX);
    show("endeffectorThetaY.command", cartesian_info.ThetaY);
    show("endeffectorThetaZ.command", cartesian_info.ThetaZ);
  }

  // Send trajectory point to arm
  kinova_jaco_api_.sendBasicTrajectory(trajectory_point);
}

void KinovaJaco::publishState() {
  auto proto_builder = tx_arm_state().initProto();
  composite::WriteSchema(state_schema_, proto_builder);

  // Allocate tensor1d to store state data
  Tensor1d state_data(7 + 2 * kNumJoints);
  int offset = 0;

  // Read cartesian pose
  CartesianPosition cartesian_pose;
  // Unit: X/Y/Z meter, ThetaX/ThetaY/ThetaZ rad, Finger1/2/3 no unit.
  kinova_jaco_api_.getCartesianPosition(cartesian_pose);
  const auto& coords = cartesian_pose.Coordinates;
  state_data(offset++) = coords.X;
  state_data(offset++) = coords.Y;
  state_data(offset++) = coords.Z;

  const Quaterniond q =
      SO3d::FromEulerAnglesRPY(coords.ThetaX, coords.ThetaY, coords.ThetaZ).quaternion();
  state_data(offset++) = q.w();
  state_data(offset++) = q.x();
  state_data(offset++) = q.y();
  state_data(offset++) = q.z();

  // Read joint position. Unit: degree
  AngularPosition joint_position;
  kinova_jaco_api_.getAngularPosition(joint_position);
  // Store observed joint position (converting from degrees to radians)
  state_data(offset++) = DegToRad(joint_position.Actuators.Actuator1);
  state_data(offset++) = DegToRad(joint_position.Actuators.Actuator2);
  state_data(offset++) = DegToRad(joint_position.Actuators.Actuator3);
  state_data(offset++) = DegToRad(joint_position.Actuators.Actuator4);
  state_data(offset++) = DegToRad(joint_position.Actuators.Actuator5);
  state_data(offset++) = DegToRad(joint_position.Actuators.Actuator6);
  state_data(offset++) = DegToRad(joint_position.Actuators.Actuator7);

  // Read joint velocities. Unit: degree/s
  AngularPosition joint_velocity;
  kinova_jaco_api_.getAngularVelocity(joint_velocity);
  // Store observed joint velocities (converting from degrees to radians)
  state_data(offset++) = DegToRad(joint_velocity.Actuators.Actuator1);
  state_data(offset++) = DegToRad(joint_velocity.Actuators.Actuator2);
  state_data(offset++) = DegToRad(joint_velocity.Actuators.Actuator3);
  state_data(offset++) = DegToRad(joint_velocity.Actuators.Actuator4);
  state_data(offset++) = DegToRad(joint_velocity.Actuators.Actuator5);
  state_data(offset++) = DegToRad(joint_velocity.Actuators.Actuator6);
  state_data(offset++) = DegToRad(joint_velocity.Actuators.Actuator7);

  for (int i = 0; i < 7; i++) {
    show("actuator" + std::to_string(i + 1) + ".position", state_data(7 + i));
    show("actuator" + std::to_string(i + 1) + ".speed", state_data(7 + kNumJoints + i));
  }

  ToProto(std::move(state_data), proto_builder.initValues(), tx_arm_state().buffers());
  tx_arm_state().publish();
}

}  // namespace kinova_jaco
}  // namespace isaac
