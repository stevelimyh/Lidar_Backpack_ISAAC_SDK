/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "KinematicTree.hpp"

#include <string>
#include <utility>
#include <vector>

#include "engine/gems/kinematic_tree/json_loader.hpp"
#include "engine/gems/kinematic_tree/kinematic_tree.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace map {

void KinematicTree::start() {
  // load the kinematic tree from file
  const auto kinematic_file = try_get_kinematic_file();
  if (!kinematic_file) {
    reportFailure("kinematic_file is required but not set in config.");
    return;
  } else if (!loadFromFile(node()->app()->getAssetPath(*kinematic_file))) {
    reportFailure("Fails to load manipulator from %s.", kinematic_file->c_str());
    return;
  }
}

const kinematic_tree::KinematicTree& KinematicTree::model() const {
  ASSERT(kinematic_tree_, "Try to access the kinematic tree object before it's loaded from file");
  return *kinematic_tree_;
}

bool KinematicTree::loadFromFile(const std::string& filename) {
  const size_t pos = filename.find_last_of(".");
  if (pos == std::string::npos) {
    return false;
  }
  const std::string file_extension = filename.substr(filename.find_last_of("."));
  kinematic_tree::KinematicTree model;
  if (file_extension == ".json") {
    const auto maybe_json = serialization::TryLoadJsonFromFile(filename);
    if (maybe_json && kinematic_tree::FromJson(*maybe_json, model)) {
      kinematic_tree_ = std::move(model);
      return true;
    } else {
      LOG_ERROR("Not a valid kinematic JSON.");
      return false;
    }
  } else {
    LOG_ERROR("File format not supported.");
    return false;
  }
}

}  // namespace map
}  // namespace isaac
