/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/sight/gems/histogram.hpp"

#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/image/image.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/sight/sop.hpp"

namespace isaac {
namespace sight {

void ShowHistogram(const VectorXd& counts, const VectorXi& highlight, int min_width, int height,
                   sight::Sop& sop) {
  ISAAC_ASSERT_GT(counts.size(), 0);
  if (highlight.size() > 0) {
    ISAAC_ASSERT_EQ(highlight.size(), counts.size());
  }

  // Histogram is rendered white on dark background with green as hightlight color
  const Pixel3ub kColorBackground{20, 20, 20};
  const Pixel3ub kColorHighlight{50, 230, 50};
  const Pixel3ub kColorNormal{220, 220, 220};

  // make bars wider in case only very few buckets are given
  int bar_width = 1;
  if (counts.size() < min_width) {
    bar_width = min_width / counts.size();
    if (bar_width * counts.size() < min_width) bar_width++;
  }

  const VectorXi heights = (static_cast<double>(height) * counts).cast<int>();

  Image3ub histogram(height, bar_width * counts.size());
  FillPixels(histogram, kColorBackground);

  // Draw each bar into the image
  for (int bucket = 0; bucket < counts.rows(); bucket++) {
    const bool is_highlight = highlight.size() > 0 && highlight[bucket] > 0;
    const Pixel3ub color = is_highlight ? kColorHighlight : kColorNormal;
    // Draw histogram by filling lines from bottom to top based on particle score
    const int bucket_rows = Clamp(heights[bucket], 0, height - 1);
    const int bucket_col_1 = bar_width * bucket;
    const int bucket_col_2 = bucket_col_1 + bar_width;
    for (int i = bucket_col_1; i < bucket_col_2; i++) {
      for (int j = 0; j < bucket_rows; j++) {
        histogram(height - 1 - j, i) = color;
      }
    }
  }

  sop.add(histogram);
}

}  // namespace sight
}  // namespace isaac
