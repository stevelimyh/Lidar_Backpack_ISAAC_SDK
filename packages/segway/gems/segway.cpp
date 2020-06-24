/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "segway.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/constants.hpp"
#include "engine/gems/serialization/bytes_convert.hpp"
#include "engine/gems/serialization/crc16.hpp"
#include "engine/gems/sight/sight.hpp"

namespace isaac {
namespace drivers {

namespace {
const std::set<std::string> kDefaultSensorReading = {
    "aux_batt_soc",      "min_propulsion_batt_soc",    "linear_accel_msp2",
    "linear_vel_mps",    "differential_wheel_vel_rps", "right_front_vel_mps",
    "left_front_vel_mps"};
}

Segway::Segway(const std::string& address, int port) {
  socket_.reset(Socket::CreateRxUDPSocket(address, port));
  serialization::crc16_initialize();

  memset(&segway_state_, 0, sizeof(segway_state_));
  // We can activate potentially 4 frames. Each frame will return different kind of data
  // Each frame uses bit masking to return information
  answers_per_frame_.push_back({});
  answers_per_frame_.push_back({});
  answers_per_frame_.push_back({});
  answers_per_frame_.push_back({});
}

void Segway::configureResponses() {
  ASSERT(socket_->isRunning(), "Cannot be called if socket not running");

  std::set<std::string> answers;

  answers.insert(kDefaultSensorReading.begin(), kDefaultSensorReading.end());
  std::vector<uint32_t> activation_code_frame = {0, 0, 0, 0};

  // Calculates the code to enable certain sensory data
  for (size_t j = 0; j < kSegwayFrames.size(); j++) {
    for (uint32_t i = 0; i < kSegwayFrames[j].size(); i++) {
      if (answers.find(std::get<0>(kSegwayFrames[j][i])) != answers.end()) {
        const std::tuple<std::string, int>& tuple = kSegwayFrames[j][i];
        activation_code_frame[j] |= 1 << i;
        answers_per_frame_[j].push_back(tuple);
      }
    }
  }

  for (size_t i = 0; i < 4; i++) {
    sendCommandU(RMP_CONFIGURATION_MESSAGE, RMP_CMD_SET_USER_FB_1_BITMAP + i,
                 activation_code_frame[i]);
  }
}

void Segway::start() {
  socket_->startSocket();
  configureResponses();
}

void Segway::stop() {
  socket_->closeSocket();
}

bool Segway::sendCommandU(uint16_t command_id, uint32_t value1, uint32_t value2) {
  rmp_set_packet_t packet;
  serialization::uint16_t_to_bytes(command_id, packet.messageId);
  serialization::uint32_t_to_bytes(value1, packet.value1);
  serialization::uint32_t_to_bytes(value2, packet.value2);
  serialization::crc16_compute_byte_buffer_crc(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  return sendCommandPacketToRmp(packet);
}

bool Segway::sendCommandF(uint16_t command_id, float value1, float value2) {
  rmp_set_packet_t packet;
  serialization::uint16_t_to_bytes(command_id, packet.messageId);
  serialization::float_to_bytes(value1, packet.value1);
  serialization::float_to_bytes(value2, packet.value2);
  serialization::crc16_compute_byte_buffer_crc(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  return sendCommandPacketToRmp(packet);
}

bool Segway::sendSpeedCommand(float linear_speed, float angular_speed) {
  // TODO Make sure that there are not magic scale factors necessary.
  return sendCommandF(0x0500, linear_speed, angular_speed);
}

bool Segway::sendCommandPacketToRmp(const rmp_set_packet_t& packet) {
  socket_->writePacket(reinterpret_cast<const void*>(&packet), sizeof(packet));
  char buffer[1024];
  int size = socket_->readPacket(buffer, sizeof(buffer));
  if (size <= 0) {
    LOG_ERROR("Command probably invalid (error %d) %02x %04x %04x %02x",
              size, packet.messageId, packet.value1, packet.value2, packet.crc);
    return false;
  }

  // Check crc
  if (!serialization::crc16_byte_buffer_crc_is_valid(reinterpret_cast<uint8_t*>(buffer), size)) {
    LOG_ERROR("Response invalid %02x %04x %04x %02x");
    return false;
  }

  int total_length = answers_per_frame_[0].size() + answers_per_frame_[1].size() +
                        answers_per_frame_[2].size() + answers_per_frame_[3].size();
  if ((size-2)/4 != total_length) {
    LOG_ERROR("Invalid response packet %d vs %d", total_length, (size-2)/4);
    return false;
  }
  int idx_answer = 0;
  // Parses successively answers for various sensors
  for (const auto& answers : answers_per_frame_) {
    for (size_t i = 0; i < answers.size(); i++) {
      serialization::bytes_to_uint32_t(reinterpret_cast<uint8_t*>(buffer) + idx_answer,
                        &rmp_data_[std::get<0>(answers[i])]);
      idx_answer += 4;
    }
  }

  for (auto answer : kDefaultSensorReading) {
    if (rmp_data_.find(answer) == rmp_data_.end()) {
      LOG_ERROR("RMP answer %s not found. Something is wrong", answer.c_str());
      return false;
    }
  }

  serialization::uint32_to_float(rmp_data_["linear_accel_msp2"], &segway_state_.linear_accel_msp2);
  serialization::uint32_to_float(rmp_data_["linear_vel_mps"], &segway_state_.linear_vel_mps);
  serialization::uint32_to_float(rmp_data_["differential_wheel_vel_rps"],
                                 &segway_state_.differential_wheel_vel_rps);
  serialization::uint32_to_float(rmp_data_["right_front_vel_mps"],
                                 &segway_state_.right_front_vel_mps);
  serialization::uint32_to_float(rmp_data_["left_front_vel_mps"],
                                 &segway_state_.left_front_vel_mps);
  serialization::uint32_to_float(rmp_data_["min_propulsion_batt_soc"],
                                 &segway_state_.min_propulsion_batt_soc);
  serialization::uint32_to_float(rmp_data_["aux_batt_soc"],
                                 &segway_state_.aux_batt_soc);
  return true;
}

}  // namespace drivers
}  // namespace isaac
