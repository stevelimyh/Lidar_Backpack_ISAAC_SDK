/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "measure.hpp"

#include <string>

#include "capnp/dynamic.h"
#include "capnp/schema.h"

#include "engine/core/assert.hpp"
#include "messages/composite.capnp.h"

namespace isaac {
namespace composite {

Measure FromProto(const ::CompositeProto::Measure measure) {
  switch (measure) {
    case ::CompositeProto::Measure::NONE:
      return Measure::kNone;
    case ::CompositeProto::Measure::TIME:
      return Measure::kTime;
    case ::CompositeProto::Measure::MASS:
      return Measure::kMass;
    case ::CompositeProto::Measure::POSITION:
      return Measure::kPosition;
    case ::CompositeProto::Measure::SPEED:
      return Measure::kSpeed;
    case ::CompositeProto::Measure::ACCELERATION:
      return Measure::kAcceleration;
    case ::CompositeProto::Measure::ROTATION:
      return Measure::kRotation;
    case ::CompositeProto::Measure::ANGULAR_SPEED:
      return Measure::kAngularSpeed;
    case ::CompositeProto::Measure::ANGULAR_ACCELERATION:
      return Measure::kAngularAcceleration;
    case ::CompositeProto::Measure::NORMAL:
      return Measure::kNormal;
    case ::CompositeProto::Measure::COLOR:
      return Measure::kColor;
    default:
      PANIC("Unknown Measure %x", measure);
  }
}

::CompositeProto::Measure ToProto(const Measure measure) {
  switch (measure) {
    case Measure::kNone:
      return ::CompositeProto::Measure::NONE;
    case Measure::kTime:
      return ::CompositeProto::Measure::TIME;
    case Measure::kMass:
      return ::CompositeProto::Measure::MASS;
    case Measure::kPosition:
      return ::CompositeProto::Measure::POSITION;
    case Measure::kSpeed:
      return ::CompositeProto::Measure::SPEED;
    case Measure::kAcceleration:
      return ::CompositeProto::Measure::ACCELERATION;
    case Measure::kRotation:
      return ::CompositeProto::Measure::ROTATION;
    case Measure::kAngularSpeed:
      return ::CompositeProto::Measure::ANGULAR_SPEED;
    case Measure::kAngularAcceleration:
      return ::CompositeProto::Measure::ANGULAR_ACCELERATION;
    case Measure::kNormal:
      return ::CompositeProto::Measure::NORMAL;
    case Measure::kColor:
      return ::CompositeProto::Measure::COLOR;
    default:
      PANIC("Unknown Measure %x", measure);
  }
}

std::optional<Measure> FromString(const std::string name) {
  const auto schema = ::capnp::Schema::from<::CompositeProto::Measure>().asEnum();
  KJ_IF_MAYBE(enumerant, schema.findEnumerantByName((::kj::StringPtr)name.c_str())) {
    const auto measure = ::capnp::DynamicEnum(*enumerant);
    return FromProto(measure.as<::CompositeProto::Measure>());
  } else {
    return std::nullopt;
  }
}

std::string ToString(const Measure measure) {
  const auto value = capnp::toDynamic(ToProto(measure));
  KJ_IF_MAYBE(enumerant, value.getEnumerant()) {
    return std::string(enumerant->getProto().getName().cStr());
  } else {
    return "";
  }
}

}  // namespace composite
}  // namespace isaac
