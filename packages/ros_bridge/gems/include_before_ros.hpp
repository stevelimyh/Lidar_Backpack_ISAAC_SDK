/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

// "boost/next_prior.h" needs to be included before ROS headers.
// ROS melodic currently expects boost 1.65
// (http://www.ros.org/reps/rep-0003.html), whereas
// Isaac has boost 1.68.
// With boost 1.67, boost::prior class used by ROS moved
// from boost/utility.hpp to boost/next_prior.hpp.
// To avoid compilation errors such as
// "boost/range/iterator_range_core.hpp:323:24: error: 'prior' is not a member of 'boost'"
// we need to include "boost/next_prior.h" before ROS headers.
#include "boost/next_prior.hpp"

