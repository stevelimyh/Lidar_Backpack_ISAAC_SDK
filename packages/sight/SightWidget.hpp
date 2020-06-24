/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace sight {

// A component which can be used to create a widget for sight.
class SightWidget : public isaac::alice::Component {
 public:
  // The type of the widget
  enum class Type {
    kInvalid,  // Not a valid widget type
    kGraphics2D,  // A 2D visualization of geometric objects
    kGraphics3D,  // A 3D visualization of geometric objects
    kTimeseries  // A variable plot of timeseries
  };

  // A sight data channel which will be shown on the widget
  struct Channel {
    // The name of the channel
    std::string name;
    // Default color of the channel
    std::string color;
    // Default size of the channel
    double size;
    // If disabled the channel will not be draw initially when the widget is created
    bool active;
  };

  // If specified the window will use this frame as the base frame (the frame used as reference when
  // rendering).
  ISAAC_PARAM(std::string, base_frame);
  // If specified the window will use this frame as the static frame (the frame used to synchronize
  // the channel with the base channel).
  ISAAC_PARAM(std::string, static_frame);
  // The caption of the widget. If not specified the component name will be used
  ISAAC_PARAM(std::string, title);
  // The type of the widget (mandatory). Possible choices are: "2d", "3d", "plot".
  ISAAC_PARAM(Type, type);
  // The initial dimensions of the widget. If not specified sight will decide.
  ISAAC_PARAM(Vector2i, dimensions);
  // A list of channels to display on the sight widget. Channels have several parameters:
  //  * name: The name of the sight channel in the form: node_name/component_name/channel_name
  //  * active: If disabled the channel will not be drawn initially when the widget is created
  ISAAC_PARAM(std::vector<Channel>, channels, {});
  // If enabled all channel names are prefixed with the app name.
  ISAAC_PARAM(bool, prepend_channel_name_with_app_name, true);
  // If enabled the title of the widget will be prefixed with the app name.
  ISAAC_PARAM(bool, prepend_title_with_app_name, true);
  // If false, this SightWidget will not create a window in Sight
  ISAAC_PARAM(bool, enabled, true);

  // Composes a JSON object which specifies the widget. The format of the object is:
  //  {
  //    "renderer": "<renderer>",
  //    "dims": { "width": <width>, "height": <height> },
  //    "channels": [
  //      { "name": <channel 1> },
  //      { "name": <channel 2>, "active": false },
  //      ...
  //      { "name": <channel n> }
  //    ]
  //  }
  // The returned pair is the desired title and the JSON object.
  std::optional<std::pair<std::string, Json>> toJson() const;
};

// Helper function to serialize an enum with json
NLOHMANN_JSON_SERIALIZE_ENUM(SightWidget::Type, {
  {SightWidget::Type::kInvalid, nullptr },
  {SightWidget::Type::kGraphics2D, "2d" },
  {SightWidget::Type::kGraphics3D, "3d" },
  {SightWidget::Type::kTimeseries, "plot" }
});

// Helper function to serialize an object to json
void to_json(Json& json, const SightWidget::Channel& channel);
// Helper function to deserialize an object from json
void from_json(const Json& json, SightWidget::Channel& channel);

}  // namespace sight
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::sight::SightWidget);
