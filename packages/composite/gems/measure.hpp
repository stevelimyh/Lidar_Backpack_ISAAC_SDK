/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/core/optional.hpp"
#include "messages/composite.capnp.h"

namespace isaac {
namespace composite {

// Possible measure for an entity in Composite Message
enum class Measure {
  kNone,                 // Unknown
  kTime,                 // Second
  kMass,                 // Kilogram
  kPosition,             // Metre
  kSpeed,                // Metre per second
  kAcceleration,         // Metre per squared second
  kRotation,             // Rotation of 2d or 3d
  kAngularSpeed,         // Radian per second
  kAngularAcceleration,  // Radian per squared second
  kNormal,               // A normal
  kColor,                // A color
};

// Parses type of measure in Proto message
Measure FromProto(const ::CompositeProto::Measure measure);

// Creates type of measure for Proto message
::CompositeProto::Measure ToProto(const Measure measure);

// Parses type of measure from string. The string should match CompositeProto::Measure, such as
// "none" and "angularSpeed".
std::optional<Measure> FromString(const std::string name);

// Convert Measure to string for json serialization.
std::string ToString(const Measure measure);

}  // namespace composite
}  // namespace isaac
