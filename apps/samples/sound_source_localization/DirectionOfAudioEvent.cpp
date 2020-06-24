/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "DirectionOfAudioEvent.hpp"

#include "engine/gems/sight/sight.hpp"
#include "engine/gems/state/io.hpp"
#include "messages/state/audio.hpp"

namespace isaac {

void DirectionOfAudioEvent::start() {
  tickOnMessage(rx_audio_angle());
  synchronize(rx_audio_energy(), rx_audio_angle());
}

void DirectionOfAudioEvent::tick() {
  messages::AudioEnergyState audio_energy_state;
  if (!FromProto(rx_audio_energy().getProto(), rx_audio_energy().buffers(), audio_energy_state)) {
    reportFailure("Received invalid proto message for audio_energy");
    return;
  }
  const double audio_energy = audio_energy_state.energy();
  show("energy", audio_energy);

  if (audio_energy >= get_energy_threshold()) {
    messages::SourceAngleState audio_angle_state;
    if (!FromProto(rx_audio_angle().getProto(), rx_audio_angle().buffers(), audio_angle_state)) {
      reportFailure("Received invalid proto message for audio_angle");
      return;
    }
    const int audio_angle_in_degrees = static_cast<int>(RadToDeg(audio_angle_state.angle()));
    show("angle_degrees", audio_angle_in_degrees);
  }
}

}  // namespace isaac
