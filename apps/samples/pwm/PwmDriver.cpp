/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "PwmDriver.hpp"

namespace isaac {
namespace pwm {

void PwmDriver::start() {
  tickPeriodically();
}

void PwmDriver::tick() {
  channelToggle_ = !channelToggle_;

  auto proto = tx_set_duty_cycle().initProto();
  proto.setChannel(0);
  proto.setDutyCycle(0.25);
  proto.setDisable(channelToggle_);  // varies between true and false
  tx_set_duty_cycle().publish();
}

}  // namespace pwm
}  // namespace isaac
