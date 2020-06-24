/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>

namespace isaac {
namespace zed {

// Evaluates time offset between two clocks of different timeframes.
// Both clock should tick on the same rate (nanoseconds for instance).
// The returned offset fits following equation:
//
// clock1() + offset = clock2() (approximately)
//
// This is an approximate computation as we work with physical clocks.

int64_t TimeOffset(const std::function<int64_t()>& clock1,
                   const std::function<int64_t()>& clock2);

}  // namespace zed
}  // namespace isaac
