/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "time_offset_calculator.hpp"

namespace {

typedef std::function<int64_t()> Clock;

// This function calculates a determinant of
// |a b|
// |c d|
constexpr int64_t Det(int64_t a, int64_t b, int64_t c, int64_t d) {
  return a * d - b * c;
}

constexpr int kNumTimestamps = 128;
constexpr int64_t kSumSquareInt = kNumTimestamps * (2 * kNumTimestamps + 1) *
                                  (kNumTimestamps + 1) / 6;
constexpr int64_t kSumInt = kNumTimestamps * (kNumTimestamps - 1) / 2;
constexpr double kDet = static_cast<double>(Det(kSumSquareInt, kSumInt,
                                             kSumInt, kNumTimestamps));

typedef std::array<int64_t, kNumTimestamps> Timestamps;

// This function performs interleaved sampling of the two clock functions and
// returns two series of timestamps
void CaptureTimestamps(const Clock& clock1, const Clock& clock2,
                       Timestamps& timestamps1, Timestamps& timestamps2) {
  // The first run of the outer loop is used to warm up CPU caches
  // On the second run the actual timestamps are stored for a future processing
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i < kNumTimestamps; ++i) {
      timestamps1[i] = clock1();
      timestamps2[i] = clock2();
    }
  }
}

// This class estimates a linear function a*x + (b + base)
// The constructor recovers the function coefficients from a series of evenly distributed samples
class LinearFunction {
 public:
  LinearFunction(const Timestamps& timestamps) {
    const int64_t C = BasedSumK(timestamps);
    const int64_t D = BasedSum(timestamps);

    const double a = static_cast<double>(Det(C, kSumInt, D, kNumTimestamps));
    const double b = static_cast<double>(Det(kSumSquareInt, C, kSumInt, D));

    this->a = a / kDet;
    this->b = b / kDet;
    base = timestamps[0];
  }

  // Linear function coefficients
  double a;
  double b;
  int64_t base;

 private:
  static int64_t BasedSum(const Timestamps& timestamps) {
    const int64_t base = timestamps[0];
    int64_t sum = 0;
    for (int64_t v : timestamps) {
      sum += v - base;
    }
    return sum;
  }

  static int64_t BasedSumK(const Timestamps& timestamps) {
    const int64_t base = timestamps[0];
    int64_t sum = 0;
    for (size_t i = 0; i < timestamps.size(); ++i) {
      sum += (i + 1) * (timestamps[i] - base);
    }
    return sum;
  }
};

}  // namespace

namespace isaac {
namespace zed {

int64_t TimeOffset(const Clock& clock1, const Clock& clock2) {
  Timestamps timestamps1;
  Timestamps timestamps2;

  CaptureTimestamps(clock1, clock2, timestamps1, timestamps2);

  const LinearFunction f1{timestamps1};
  const LinearFunction f2{timestamps2};

  return f2.base - f1.base + static_cast<int64_t>(f2.b - (f1.b + f1.a / 2));
}

}  // namespace zed
}  // namespace isaac
