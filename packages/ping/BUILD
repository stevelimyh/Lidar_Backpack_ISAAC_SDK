load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
  name = "ping_components",
  srcs = [
      "Ping.cpp",
      "Pong.cpp"
      ],
  hdrs = [
      "Ping.hpp",
      "Pong.hpp"
      ]
)

isaac_app(
     name = "ping",
     modules = [
         "//packages/ping:ping_components",
     ]
)