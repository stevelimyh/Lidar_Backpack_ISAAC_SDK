#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

# constants
COLOR_RED='\033[0;31m'
COLOR_NONE='\033[0m'

# Helper functions to be used in this script.
# Prints the error message and exits the script.
error_and_exit() {
  printf "${COLOR_RED}Error: $1${COLOR_NONE}\n"
  exit 1
}

while getopts :d: option
do
  case ${option} in
    d) DEVICE=${OPTARG};;
    *) break;
  esac
done
shift $((OPTIND -1))

if [ -z $DEVICE ]
then
  error_and_exit "Device must be specified with -d flag. Valid choices are 'xavier' and 'x64'."
fi

engine/alice/tools/main --app apps/samples/stereo_depth_dnn/stereo_depth_dnn.app.json \
--config "apps/samples/stereo_depth_dnn/stereo_depth_dnn.config.$DEVICE.json"

