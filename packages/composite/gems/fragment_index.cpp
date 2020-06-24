/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "fragment_index.hpp"

#include <algorithm>
#include <utility>

#include "engine/core/assert.hpp"

namespace isaac {
namespace composite {

std::optional<FragmentIndex> FragmentIndex::Create(const Schema& source, const Schema& relevant,
                                                   const Schema& target) {
  FragmentIndex result;
  result.source_count_ = source.getElementCount();
  result.target_count_ = target.getElementCount();
  result.fragments_.reserve(relevant.getQuantities().size());

  for (const Quantity& quantity : relevant.getQuantities()) {
    const auto maybe_source_index = source.findQuantityValueIndex(quantity);
    const auto maybe_target_index = target.findQuantityValueIndex(quantity);
    if (maybe_source_index == std::nullopt || maybe_target_index == std::nullopt) {
      return std::nullopt;
    }
    result.fragments_.push_back(
        {*maybe_source_index, *maybe_target_index, quantity.getElementCount()});
  }

  return result;
}

void FragmentIndex::copyFragments(const double* source_ptr, int source_count,
                                  double* target_ptr, int target_count) const {
  ASSERT(source_count == source_count_, "invalid source length");
  ASSERT(target_count == target_count_, "invalid target length");
  for (const auto& fragment : fragments_) {
    const int source_offset = fragment.source_offset;
    const int target_offset = fragment.target_offset;
    const int count = fragment.count;
    std::copy(source_ptr + source_offset, source_ptr + source_offset + count,
              target_ptr + target_offset);
  }
}

}  // namespace composite
}  // namespace isaac
