#!/usr/bin/env bash

# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Script to build the C-API stand alone example application
# usage: copy the sample application files into your deployed C-API folder,
#        then from the deployed C-API folder run ./build.sh

# if c_api_example exists, remove it
if [ -f "./c_api_example" ]; then
  rm ./c_api_example
fi

# compile c_api_example
gcc c_api_example.c \
    -L./engine/alice/c_api -lisaac_c_api \
    -L./packages/sight -lsight_module \
    -o c_api_example

# if c_api_example exists, run it
if [ -f "./c_api_example" ]; then
  # export library paths
  export LD_LIBRARY_PATH=./engine/alice/c_api:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=./packages/sight:$LD_LIBRARY_PATH

  # run the example
  ./c_api_example
fi
