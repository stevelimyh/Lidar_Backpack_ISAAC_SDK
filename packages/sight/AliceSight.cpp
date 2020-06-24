/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "AliceSight.hpp"

#include <string>
#include <utility>

#include "engine/gems/sight/sight.hpp"

namespace isaac {
namespace sight {

void AliceSight::start() {
  // Setup raw sight so that it uses alice's websight
  ResetSight(this);
}

void AliceSight::stop() {
  // Unregisters raw sight
  ResetSight(nullptr);
}

void AliceSight::plotValue(const std::string& name, double time, float value) {
  show(name, time, value);
}

void AliceSight::plotValue(const std::string& name, double time, double value) {
  show(name, time, value);
}

void AliceSight::plotValue(const std::string& name, double time, int value) {
  show(name, time, value);
}

void AliceSight::plotValue(const std::string& name, double time, int64_t value) {
  show(name, time, value);
}

void AliceSight::log(const char* file, int line, logger::Severity severity, const char* log,
                     double time) {
  // FIXME implement
}

void AliceSight::drawCanvas(const std::string& name, sight::Sop sop) {
  show(name, std::move(sop));
}

}  // namespace sight
}  // namespace isaac
