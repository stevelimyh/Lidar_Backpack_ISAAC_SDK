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
#include <vector>

#include "KinovaTypes.h"
#include "engine/alice/alice_codelet.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/kinematic_tree/kinematic_tree.hpp"
#include "messages/composite.capnp.h"
#include "packages/composite/gems/parser.hpp"
#include "packages/kinova_jaco/gems/kinova_jaco_api.hpp"

namespace isaac {
namespace kinova_jaco {

// A class to receive command and publish state information for the Kinova Jaco arm.
class KinovaJaco : public alice::Codelet {
 public:
  // Available control modes for arm.
  enum class ControlMode {
    kJointPosition,        // Controls position for each joint
    kJointVelocity,        // Controls velocity for each joint
    kEndEffectorPose,      // Control end effector 3D pose
    kEndEffectorVelocity,  // Control end effector translation and rotation velocity
    kInvalid = -1          // Invalid control mode, returned from an invalid string in JSON
  };

  void start() override;
  void tick() override;
  void stop() override;

  // Command for the arm, parsed based on control mode.
  // In joint control mode, requires a scalar quantity for each of the seven joints with entity
  // matching the kinematic tree link names, and measure matching the control mode (position or
  // speed).
  // In end effector control, the entity name matching the end_effector_name set in config. In
  // end effector pose control, expects a position quantity of translation vector(Vector3) and a
  // rotation quantity of quaternion (Vector4), in base coordinate. In end effector velocity
  // control, expects a speed quantity of linear velocity (Vector3) in base coordinate and a
  // angularSpeed quantity of angular speed (Vector3) in end-effector coordinate.
  ISAAC_PROTO_RX(CompositeProto, arm_command);

  // Current state for the arm, includes end effector pose, joint positions and velocities. The
  // schema includes the following:
  // end effector position (Vector3) and rotation quaternion (Vector4), in base coordinate;
  // a scalar quantity for position and for speed for each of the seven joints, entity name is
  // the link name in kinematic tree;
  ISAAC_PROTO_TX(CompositeProto, arm_state);

  // Path to JacoSDK is set in jaco_driver_config.json.
  // Driver is tested for use with `JACO2SDK v1.4.2`_
  //
  // .. _JACO2SDK v1.4.2: https://drive.google.com/file/d/17_jLW5EWX9j3aY3NGiBps7r77U2L64S_/view
  ISAAC_PARAM(std::string, kinova_jaco_sdk_path);
  // If true, initializes arm and finger to home position in start.
  ISAAC_PARAM(bool, initialize_home, false);
  // Set control mode for arm. This can be changed at runtime
  ISAAC_PARAM(ControlMode, control_mode, ControlMode::kJointPosition);
  // Name of the node containing the map:KinematicTree component
  ISAAC_PARAM(std::string, kinematic_tree);
  // Name of the end effector for parsing/creating composite message.
  ISAAC_PARAM(std::string, end_effector_name, "end_effector");

 private:
  // Validates the kinematic tree object is consistent with the hardware model
  bool validateKinematicTree(const kinematic_tree::KinematicTree& model);

  void parseCommand();
  // Generates a schema for parsing command based on control mode
  void initCommandParser(const ControlMode mode);
  // Parse command from Composite message

  // Query current arm state
  void publishState();
  // Generate the composite schema for publishing state
  void initStateSchema();

  // Interface to Jaco API functions
  KinovaJacoAPI kinova_jaco_api_;
  // True if API has been successfully initialized
  bool successful_api_open_;
  // Cache control mode. If control mode changed, need to reset parser schema.
  ControlMode control_mode_ = ControlMode::kInvalid;
  // Cache list of joint names used in the kinematic tree
  std::vector<std::string> joint_names_;

  // Acquire time for most recently received position command
  std::optional<int64_t> last_command_time_;
  // Parsers for arm command message
  composite::Parser command_parser_;
  // Cache schema for arm state message
  composite::Schema state_schema_;
};

// Mapping between each ControlMode type and an identifying string.
NLOHMANN_JSON_SERIALIZE_ENUM(KinovaJaco::ControlMode, {
  {KinovaJaco::ControlMode::kJointPosition, "joint position"},
  {KinovaJaco::ControlMode::kJointVelocity, "joint velocity"},
  {KinovaJaco::ControlMode::kEndEffectorPose, "end effector pose"},
  {KinovaJaco::ControlMode::kEndEffectorVelocity, "end effector velocity"},
  {KinovaJaco::ControlMode::kInvalid, nullptr}
});

}  // namespace kinova_jaco
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::kinova_jaco::KinovaJaco);
