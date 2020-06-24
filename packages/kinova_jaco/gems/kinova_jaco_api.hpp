/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "KinovaTypes.h"

#include <string>

namespace isaac {
namespace kinova_jaco {

// Jaco API class to interface with the Jaco arm over USB
class KinovaJacoAPI {
 public:
  // Load Jaco SDK and initialize USB connection with the arm
  bool open(const std::string& kinova_jaco_sdk_path);

  // Close connection with Jaco arm
  bool close();

  // Functions use to interact with the Kinova SDK:
  int (*initAPI)();
  int (*closeAPI)();
  int (*initFingers)();
  int (*moveHome)();
  int (*getCartesianPosition)(CartesianPosition& position);
  int (*getAngularPosition)(AngularPosition& joint_angles);
  int (*sendBasicTrajectory)(TrajectoryPoint trajectory);
  int (*setCartesianControl)();
  int (*startControlAPI)();
  int (*eraseAllTrajectories)();
  int (*getCartesianForce)(CartesianPosition& force);
  int (*getGlobalTrajectoryInfo)(TrajectoryFIFO& Response);
  int (*getAngularVelocity)(AngularPosition& joint_vel);

 private:
  // Loads a function from the command layer
  template <typename... Args>
  int (*loadFunction(const std::string& name))(Args...);  // NOLINT

  // A pointer to the command layer dll.
  void* command_layer_handle_;
};

}  // namespace kinova_jaco
}  // namespace isaac
