/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/audio_file_playback.capnp.h"

namespace isaac {

// Play an audio file periodically
class AudioPlaybackFileIndex : public alice::Codelet {
 public:
  // Publish file index
  ISAAC_PROTO_TX(AudioFilePlaybackProto, audio_fileindex)
  // File index
  ISAAC_PARAM(int, file_index, 0)

  void start() override;
  void tick() override;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::AudioPlaybackFileIndex);
