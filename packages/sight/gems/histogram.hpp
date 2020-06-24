/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/types.hpp"
#include "engine/gems/sight/sop.hpp"

namespace isaac {
namespace sight {

// Shows a histogram with sight
// Every element in `counts` will result in one bar. Values are taken as is and are not rescaled.
// A value of 1 indicate maximum height. `highlight` can be used to indicate which bars in the
// historam are highlighted. Currently 0 means no highlight, and 1 means highlighted. The histogram
// will be shown as an image which has at least the width `min_width` and the height `height`.
void ShowHistogram(const VectorXd& counts, const VectorXi& highlight, int min_width, int height,
                   sight::Sop& sop);

}  // namespace sight
}  // namespace isaac
