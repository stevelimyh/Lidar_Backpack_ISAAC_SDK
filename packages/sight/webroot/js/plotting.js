/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

function size_dict(d){c=0; for (i in d) ++c; return c}
function getSlicingIndex(name) {
  let slash_pos = name.lastIndexOf("/");
  let dot_pos = name.lastIndexOf(".");
  return Math.max(slash_pos, dot_pos);
}
function sightPlotHandle(lists) {
    if (lists == null) {
      return;
    }
    const newlist = (lists instanceof Array) ? list : {lists};
    for (var id in newlist) {
      if (newlist[id]["type"] != "plot") {
        return;
      }
      let list = newlist[id];
      let name = channel_mapping_[list.uuid];
      if (name === undefined) continue;
      if (channel_enabled_[name] !== true) continue;
      let slicing_index = getSlicingIndex(name);
      let plot = name.slice(slicing_index + 1);
      let win = name.slice(0, slicing_index);
      let value = parseFloat(list.v);

      if (!(name in plots_)) {
        plots_[name] = new TimeSeries();
      }
      // Multiply the timestamp by 1000 as Smoothie libs use milli seconds while ISAAC sends seconds
      plots_[name].append(list.t * 1000.0, value, false, true);
  }
};
var plots_ = {};
var windows_ = {};
