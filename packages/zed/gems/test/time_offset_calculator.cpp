/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/zed/gems/time_offset_calculator.hpp"

#include <chrono>

#include "gtest/gtest.h"

namespace {

int64_t std_steady_clock() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()
  ).count();
}

} // namespace


TEST(time_offset_calculator, same_cpp_clock) {
  const int64_t offset = isaac::zed::TimeOffset(std_steady_clock, std_steady_clock);
  EXPECT_LT(-100, offset     );
  EXPECT_LT(      offset, 100);
}


TEST(time_offset_calculator, shifted_cpp_clocks) {
  const int64_t shift1 =  12345678;
  const int64_t shift2 = -87654321;
  const int64_t offset = isaac::zed::TimeOffset(
      [](){ return std_steady_clock() + shift1;},
      [](){ return std_steady_clock() + shift2;}
    );
  EXPECT_LT(-100, offset + 99999999     );
  EXPECT_LT(      offset + 99999999, 100);
}
