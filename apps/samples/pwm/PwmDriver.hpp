/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>

#include "engine/alice/alice_codelet.hpp"
#include "messages/pwm_channel_set.capnp.h"

namespace isaac {
namespace pwm {

// Codelet to toggle PWM channel 0 on and off
class PwmDriver : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // used to set a duty cycle for a PWM channel
  ISAAC_PROTO_TX(PwmChannelSetDutyCycleProto, set_duty_cycle);

 private:
  // toggle the channel off and on
  bool channelToggle_ = false;
};

}  // namespace pwm
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::pwm::PwmDriver);
