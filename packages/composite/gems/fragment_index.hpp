/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/core/optional.hpp"
#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace composite {

// A helper type to copy fragments between schemas
class FragmentIndex {
 public:
  // Identifies relevant fragments in source and target
  static std::optional<FragmentIndex> Create(const Schema& source, const Schema& relevant,
                                             const Schema& target);

  // Empty index
  FragmentIndex() = default;

  // Copies fragments from source buffer to target buffer
  void copyFragments(const double* source_ptr, int source_count,
                     double* target_ptr, int target_count) const;

 private:
  // Locations of a bunch of elementsin the source and int the target sequence
  struct Fragment {
    // Index at which the first element of the fragment is stored in the source sequence
    int source_offset;
    // Index at which the first element of the fragment is stored in the target sequence
    int target_offset;
    // Number of elements in the fragment
    int count;
  };

  // Number of elements in the source schema (used for checks)
  int source_count_ = 0;

  // Number of elements in the target schema (used for checks)
  int target_count_ = 0;

  // Fragments to copy between schema
  std::vector<Fragment> fragments_;
};

}  // namespace composite
}  // namespace isaac
