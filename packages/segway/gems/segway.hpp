/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "engine/core/constants.hpp"
#include "engine/gems/coms/socket.hpp"
#include "packages/segway/gems/rmp_types.hpp"

namespace isaac {
namespace drivers {

// The state of the Segway
struct SegwayState {
  // Linear acceleration derived from wheel velocities
  float linear_accel_msp2;
  // Linear velocity
  float linear_vel_mps;
  // Differential wheel speed (yaw rate) of the RMP derived using wheel velocities
  float differential_wheel_vel_rps;
  // Right front wheel velocity
  float right_front_vel_mps;
  // Left front wheel velocity
  float left_front_vel_mps;
  // The minimum of all propulsion battery states of charge (Percentage)
  float min_propulsion_batt_soc;
  // The auxiliary battery state of charge (Percentage)
  float aux_batt_soc;
};

// Segway RMP class that interface with the robot base over Ethernet
class Segway {
 public:
  // Instantiates a Segway with specified ip address and port
  Segway(const std::string& address, int port);

  // Starts listening
  void start();

  // Stops listening
  void stop();

  // Sends a speed command to the segway (linear speed and angular speed)
  bool sendSpeedCommand(float linear_speed, float angular_speed);

  // Sends a Float to the robot
  bool sendCommandF(uint16_t command_id, float value1, float value2);

  // Sends a Unsigned int to the robot
  bool sendCommandU(uint16_t command_id, uint32_t value1, uint32_t value2);

  // Sends a command to the Robor
  bool sendCommandPacketToRmp(const rmp_set_packet_t& packet);

  // Configures which responses we would like to receive
  void configureResponses();

  // Returns current RMP state
  const SegwayState& getSegwayState() {
    return segway_state_;
  }

 private:
  std::unique_ptr<Socket> socket_;
  std::map<std::string, uint32_t> rmp_data_;
  std::vector<std::vector<std::tuple<std::string, int>>> answers_per_frame_;
  SegwayState segway_state_;
};

}  // namespace drivers
}  // namespace isaac
