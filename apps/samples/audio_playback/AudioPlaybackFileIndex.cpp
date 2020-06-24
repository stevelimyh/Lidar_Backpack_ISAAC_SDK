/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "AudioPlaybackFileIndex.hpp"

namespace isaac {

void AudioPlaybackFileIndex::start() {
  tickPeriodically();
}

void AudioPlaybackFileIndex::tick() {
  auto builder = tx_audio_fileindex().initProto();
  builder.setFileIndex(get_file_index());
  tx_audio_fileindex().publish(getTickTimestamp());
}

}  // namespace isaac
