/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "kinova_jaco_api.hpp"

#include <dlfcn.h>
#include <string>

#include "engine/core/logger.hpp"
#include "Kinova.API.USBCommandLayerUbuntu.h"
#include "Kinova.API.USBCommLayerUbuntu.h"

namespace isaac {
namespace kinova_jaco {

bool KinovaJacoAPI::open(const std::string& kinova_jaco_sdk_path) {
  // Load in the usb command dll.
  const std::string command_layer_dll_name =
          kinova_jaco_sdk_path + "Kinova.API.USBCommandLayerUbuntu.so";
  command_layer_handle_ = dlopen(command_layer_dll_name.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (!command_layer_handle_) {
    LOG_ERROR("Failed to get command layer handle");
    return false;
  }

  // Get functions from the dll.
  initAPI = loadFunction("InitAPI");
  closeAPI = loadFunction("CloseAPI");
  initFingers = loadFunction("InitFingers");
  moveHome = loadFunction("MoveHome");
  getCartesianPosition = loadFunction<CartesianPosition&>("GetCartesianPosition");
  getAngularPosition = loadFunction<AngularPosition&>("GetAngularPosition");
  sendBasicTrajectory = loadFunction<TrajectoryPoint>("SendBasicTrajectory");
  setCartesianControl = loadFunction("SetCartesianControl");
  startControlAPI = loadFunction("StartControlAPI");
  eraseAllTrajectories = loadFunction("EraseAllTrajectories");
  getCartesianForce = loadFunction<CartesianPosition&>("GetCartesianForce");
  getGlobalTrajectoryInfo = loadFunction<TrajectoryFIFO&>("GetGlobalTrajectoryInfo");
  getAngularVelocity = loadFunction<AngularPosition&>("GetAngularVelocity");

  // Initialize the Kinova SDK.
  const int init_result = initAPI();
  if (init_result == ERROR_LOAD_COMM_DLL) {
    LOG_ERROR("Cannot find Kinova.API.CommLayerUbuntu.so. "
              "Please copy Kinova.API.CommLayerUbuntu.so to /usr/lib");
    return false;
  } else if (init_result != NO_ERROR_KINOVA) {
    LOG_ERROR("Failed to init kinova api (error %d)", init_result);
    return false;
  } else {
    return true;
  }
}

bool KinovaJacoAPI::close() {
  if (command_layer_handle_) {
    int close_result = closeAPI();
    if (close_result != NO_ERROR_KINOVA) {
      LOG_ERROR("Failed to close kinova api (error %d)", close_result);
      return false;
    }

    close_result = dlclose(command_layer_handle_);
    if (close_result) {
      LOG_ERROR("Failed to close command layer handle (error %d)", close_result);
      return false;
    }

    command_layer_handle_ = nullptr;
  }
  return true;
}

template <typename... Args>
int (*KinovaJacoAPI::loadFunction(const std::string& name))(Args...) {    // NOLINT
  auto* function = reinterpret_cast<int (*)(Args...)>(dlsym(command_layer_handle_, name.c_str()));
  if (!function) {
    LOG_ERROR("Failed to load function: %s", name.c_str());
    return nullptr;
  }
  return function;
}

}  // namespace kinova_jaco
}  // namespace isaac

