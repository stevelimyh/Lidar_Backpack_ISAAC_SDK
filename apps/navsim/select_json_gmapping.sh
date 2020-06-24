#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

# Parameter "-r" represents the robot ID (to determine which robot is being used).
# Both of them have default values in case they're not provided

while getopts :r: option
do
  case ${option} in
    r) ROBOT_ID=${OPTARG};;
    *) break;
  esac
done
shift $((OPTIND -1))

if [ -z $ROBOT_ID ]
then
  ROBOT_ID="carter"
fi

engine/alice/tools/main --app apps/navsim/navsim_sub_gmap.app.json \
--more "packages/navsim/robots/$ROBOT_ID.json" \
 $@

