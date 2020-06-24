/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SightWidget.hpp"

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace sight {

std::optional<std::pair<std::string, Json>> SightWidget::toJson() const {
  if (!get_enabled()) {
    return std::nullopt;
  }

  Json json;

  // The type of the widget is mandatory
  if (const auto maybe_type = try_get_type()) {
    json["renderer"] = *maybe_type;
  } else {
    LOG_ERROR("Mandatory parameter 'type' not specified");
    return std::nullopt;
  }

  // If specified set the desired dimensions for the widget
  if (const auto maybe = try_get_dimensions()) {
    json["dims"]["height"] = (*maybe)[0];
    json["dims"]["width"] = (*maybe)[1];
  }

  if (const auto maybe = try_get_static_frame()) {
    json["static_frame"] = *maybe;
    json["custom_frame"] = true;
  }

  if (const auto maybe = try_get_base_frame()) {
    json["base_frame"] = *maybe;
    json["custom_frame"] = true;
  }

  // Get a list of all channels which should be displayed in the widget
  const std::string app_name = node()->app()->name();
  const bool prepend_channel_name_with_app_name = get_prepend_channel_name_with_app_name();
  Json channels_json;
  for (const auto& channel : get_channels()) {
    Json channel_json;
    channel_json["name"] =
        prepend_channel_name_with_app_name ? app_name + "/" + channel.name : channel.name;
    if (!channel.active) {
      channel_json["active"] = false;
    }
    if (!channel.color.empty()) {
      channel_json["config"]["color"] = channel.color;
    }
    if (channel.size > 0.0) {
      channel_json["config"]["size"] = channel.size;
    }
    channels_json.push_back(std::move(channel_json));
  }
  json["channels"] = std::move(channels_json);

  // Determine the desired title for the widget
  std::string title = try_get_title().value_or(name());
  if (get_prepend_title_with_app_name()) {
    title = app_name + " - " + title;
  }

  return std::pair<std::string, Json>{title, json};
}

void to_json(Json& json, const SightWidget::Channel& channel) {
  json = Json{{"name", channel.name}, {"active", channel.active}};
  if (!channel.color.empty()) {
    json["color"] = channel.color;
  }
  if (channel.size > 0.0) {
    json["size"] = channel.size;
  }
}

void from_json(const Json& json, SightWidget::Channel& channel) {
  json.at("name").get_to(channel.name);
  channel.active = json.value("active", true);
  channel.color = json.value("color", "");
  channel.size = json.value("size", -1.0);
}

}  // namespace sight
}  // namespace isaac
