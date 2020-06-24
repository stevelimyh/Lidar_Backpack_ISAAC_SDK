/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/state.capnp.h"

namespace isaac {

// This Codelet receives the average audio energy of the audio packets and the azimuth angle of
// the direction of dominant sound source in the audio packets and displays the azimuth angle of
// only the audio packets whose average audio energy is higher than the configured threshold.
class DirectionOfAudioEvent : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Receive the average audio energy of the audio packet in dB for comparing with the threshold.
  ISAAC_PROTO_RX(StateProto, audio_energy);
  // Receive the azimuth angle in radians for the dominant sound source in the audio packet.
  ISAAC_PROTO_RX(StateProto, audio_angle);

  // Audio energy threshold in dB
  ISAAC_PARAM(double, energy_threshold, -80.0);
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::DirectionOfAudioEvent);
