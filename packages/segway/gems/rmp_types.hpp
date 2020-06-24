/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <tuple>
#include <vector>

namespace isaac {
namespace drivers {

// Please refer to Segway documentation
// https://1l3kf73dl0pg27adzc3ob3r9-wpengine.netdna-ssl.com/wp-content/uploads/2014/09/RMP-220.pdf
#pragma pack(push, 1)
// Set packet to move the robot
struct rmp_set_packet_t {
  uint8_t messageId[2];
  uint8_t value1[4];
  uint8_t value2[4];
  uint8_t crc[2];
};
#pragma pack(pop)


// COMMANDS
// First bitmap (contains configurable sensory data) buffer 1 uint32_t
constexpr uint32_t RMP_CMD_SET_USER_FB_1_BITMAP = 17;
// First bitmap (contains configurable sensory data) buffer 2 uint32_t
constexpr uint32_t RMP_CMD_SET_USER_FB_2_BITMAP = 18;
// First bitmap (contains configurable sensory data) buffer 3 uint32_t
constexpr uint32_t RMP_CMD_SET_USER_FB_3_BITMAP = 19;
// First bitmap (contains configurable sensory data) buffer 4 uint32_t
constexpr uint32_t RMP_CMD_SET_USER_FB_4_BITMAP = 20;

// OPERATION MODE
enum OperationalMode {
  DISABLE_REQUEST = 1,
  POWERDOWN_REQUEST = 2,
  STANDBY_REQUEST = 4,
  TRACTOR_REQUEST = 5
};

constexpr uint32_t RMP_CMD_SET_OPERATIONAL_MODE = 32;
// Reset the integrators uint32_t
constexpr uint32_t RMP_CMD_RESET_INTEGRATORS = 34;
// Sets Audio command uint32_t 0 to 16
constexpr uint32_t RMP_CMD_SET_AUDIO_COMMAND = 31;

// MESSAGES ID
// Velocity message id (velocity, yaw rate) between -1.0 and 1.0
constexpr uint32_t RMP_VELOCITY_MESSAGE = 0x0500;
// Configuration command message id
constexpr uint32_t RMP_CONFIGURATION_MESSAGE = 0x0501;

// User Defined Feedback Bitmap 1
const std::vector<std::tuple<std::string, int>> frame1 = {
  std::make_tuple("fault_status_word_1", 0),  // Unitless Fault status word 1
  std::make_tuple("fault_status_word_2", 0),  // Unitless Fault status word 2
  std::make_tuple("fault_status_word_3", 0),  // Unitless Fault status word 3
  std::make_tuple("fault_status_word_4", 0),  // Unitless Fault status word 4
  std::make_tuple("mcu_0_fault_status", 0),   // Unitless MCU 0 internal fault status
  std::make_tuple("mcu_1_fault_status1", 0),  // Unitless MCU 1 internal fault status
  std::make_tuple("mcu_2_fault_status1", 0),  // Unitless MCU 2 internal fault status
  std::make_tuple("mcu_3_fault_status1", 0),  // Unitless MCU 3 internal fault status
  // Float32 Seconds The operational runtime in seconds since the last power on
  std::make_tuple("frame_count", 1),
  std::make_tuple("operational_state", 0),    // Unitless CCU Init: 0
  std::make_tuple("dynamic_response", 0),     // Unitless No Response: 0x00000000
  // Percentage The minimum of all propulsion battery states of charge
  std::make_tuple("min_propulsion_batt_soc", 1),
  std::make_tuple("aux_batt_soc", 1),         // Percentage The auxiliary battery state of charge
  std::make_tuple("inertial_x_acc_g", 1),     // The raw x axis acceleration
  std::make_tuple("inertial_y_acc_g", 1),     // The raw y axis acceleration
  std::make_tuple("inertial_x_rate_rps", 1),  // rad/s The raw x rotational rate
  std::make_tuple("inertial_y_rate_rps", 1),  // rad/s The raw y rotational rate
  std::make_tuple("inertial_z_rate_rps", 1),  // rad/s The raw z rotational rate
  std::make_tuple("pse_pitch_deg2", 1),       // deg The estimated inertial pitch angle
  std::make_tuple("pse_pitch_rate_dps", 1),   // deg/s The estimated inertial pitch rate
  std::make_tuple("pse_roll_deg2", 1),        // deg The estimated inertial roll angle
  std::make_tuple("pse_roll_rate_dps", 1),    // deg/s The estimated inertial roll rate
  std::make_tuple("pse_yaw_rate_dps", 1),     // deg/s The estimated inertial yaw rate
  // Unit-less This is a bitmap of valid PSE data. There are two PSEs running on the CCU: one for
  // each redundant side of the BSA. If the value is zero, 0),PSE data should be discarded.
  std::make_tuple("pse_data_is_valid", 0),
  // rad/s The machine yaw rate limit, including internal limits set by the Safety Kernel
  std::make_tuple("yaw_rate_limit_rps", 1),
  // m/s The machine velocity limit,including internal limits set by the Safety Kernel
  std::make_tuple("vel_limit_mps", 1),
  std::make_tuple("linear_accel_msp2", 1),    // m/s2 Linear accel. derived from wheel velocities
  std::make_tuple("linear_vel_mps", 1),       // m/s Linear velocity of the RMP
  // Float32 rad/s Differential wheel speed (yaw rate) of the RMP derived using wheel velocities
  std::make_tuple("differential_wheel_vel_rps", 1),
  std::make_tuple("right_front_vel_mps", 1),  // Float32 m/s Right front wheel velocity
  std::make_tuple("left_front_vel_mps", 1),   // Float32 m/s Left front wheel velocity
  std::make_tuple("right_rear_vel_mps", 1),   // Float32 m/s Right rear wheel velocity
};

// User Defined Feedback Bitmap 2
// Feel out this array with correct parameters (from the doc)
const std::vector<std::tuple<std::string, int>> frame2 = {};

// Helper container to iterate over the frames
const std::vector<std::vector<std::tuple<std::string, int>>> kSegwayFrames
    = {frame1, frame2, {}, {}};

}  // namespace drivers
}  // namespace isaac
