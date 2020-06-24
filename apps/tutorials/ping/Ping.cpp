/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Ping.hpp"

namespace isaac {

void Ping::start() {
  // This part will be run once in the beginning of the program

  // We can tick periodically, on every message, or blocking. The tick period is set in the
  // json ping.app.json file. You can for example change the value there to change the tick
  // frequency of the node.
  // Alternatively you can also overwrite configuration with an existing configuration file like
  // in the example file fast_ping.json. Run the application like this to use an additional config
  // file:
  //   bazel run //apps/tutorials/ping -- --config apps/tutorials/ping/fast_ping.json
  tickPeriodically();
}

void Ping::tick() {
  // This part will be run at every tick. We are ticking periodically in this example.

  // Print the desired message to the console
  LOG_INFO(get_message().c_str());
}

}  // namespace isaac
