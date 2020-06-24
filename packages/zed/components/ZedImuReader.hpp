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
#include "messages/imu.capnp.h"
#include "messages/math.capnp.h"
#include "sl/Camera.hpp"

namespace isaac {
class ZedCamera;

namespace zed {

// Publishes IMU readings obtained from a ZED Mini camera
// Requires a ZedCamera component to exist in the same node
class ZedImuReader : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // IMU data (if available)
  // This is performed on every tick, so IMU poll rate is equal to the codelet tick frequency
  ISAAC_PROTO_TX(ImuProto, imu_raw);

  // ZED SDK <= 2.8.3 has a bug - the reported IMU translation is incorrectly scaled by 1.0e-3
  // https://github.com/stereolabs/zed-examples/issues/192
  ISAAC_PARAM(double, imu_translation_scaling_factor, 1.0e3);
  // The IMU frame used to define the left_camera_T_imu transform in the PoseTree
  ISAAC_PARAM(std::string, imu_frame, "zed_imu");

 private:
  void publishImuData(const sl::IMUData& imu_data);

  ZedCamera* camera_holder_;
  bool left_camera_T_imu_initialized_;
};

}  // namespace zed
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::zed::ZedImuReader);
