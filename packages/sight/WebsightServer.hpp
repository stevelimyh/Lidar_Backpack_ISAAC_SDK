/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/backend/sight_backend.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/algorithm/flow_control.hpp"
#include "engine/gems/math/exponential_moving_average.hpp"
#include "uWS.h"

namespace isaac {
namespace alice {
class SightChannelStatus;
}  // namespace alice
}  // namespace isaac

namespace isaac {
namespace sight {

// The webSightServer class serves the frontend web visualization. Data is sent over a websocket
// defined by a predefined API.
class WebsightServer : public isaac::alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // Port for the communication between web server and Sight
  ISAAC_PARAM(int, port, 3000);
  // Path to the files needed for Sight
  ISAAC_PARAM(std::string, webroot, "packages/sight/webroot");
  // Path to assets used by the webpage like for example pictures or robot model.
  ISAAC_PARAM(std::string, assetroot, "../isaac_assets");
  // Bandwidth to limit the rate of data transfer
  ISAAC_PARAM(int, bandwidth, 10000000);  // TODO(claire) Change to something smaller
  // Whether to compress data for transfer
  ISAAC_PARAM(bool, use_compression, true);
  // Configuration for User Interface (UI)
  ISAAC_PARAM(nlohmann::json, ui_config, (nlohmann::json{{"windows", {}}}));

 private:
  struct Connection {
    std::set<std::string> enabled_tags;
    FlowControl<std::string> flow_control;
  };

  // Statistics about how much data a channel sends. Note that data is only send if the channel is
  // enabled in at least one websight frontend.
  struct ChannelStatistics {
    // Bandwidth used by the channel in MB/s
    math::ExponentialMovingAverageRate<double> current_bandwidth;
    // Total data sent in MB
    double total = 0.0;
    // Number of items published
    size_t count = 0;
  };

  // Creates the getLatest message (latest message of each channels) for a given client
  Json getLatestMessages(uWS::WebSocket<uWS::SERVER>* ws);
  // Adds a new tag to the list
  void addTags(const std::string& name);
  // Creates the getAllTags message for a given client
  Json getAllTagsMessage(uWS::WebSocket<uWS::SERVER>* ws);
  // Handles the "setTagsStatus" message
  void setTagsStatusHandle(const Json& json_in, uWS::WebSocket<uWS::SERVER>* ws);
  // Gets the file name and is_binary flag for a path requested by the frontend
  std::string getResourceFilename(const std::string& request_path);
  // Loads a resource and returns a pointer to it
  const std::vector<char>& loadResource(const std::string& filename);
  // Loads a compressed resource and returns a pointer to it. If it fails to load the compressed
  // version, compression_used will be false and the uncompressed version will be returned.
  const std::vector<char>& loadCompressedResource(const std::string& filename,
                                                  bool& compression_used);
  // Sends JSON to a client
  void sendJson(const Json& json, uWS::WebSocket<uWS::SERVER>* ws) const;
  // Send the UI configuration to a client
  Json getUiConfigJson() const;

  // Checks wether this is an endpoint to forward to the frontend
  bool isReplyEndpoint(const alice::MessageLedger::Endpoint& endpoint) const;

  // Processes a new JSON message
  void processNewJsonMessage(const alice::MessageLedger::Endpoint& endpoint,
                             const alice::JsonMessage* message);
  // Processes a new Sight message
  void processNewSightMessage(const alice::MessageLedger::Endpoint& endpoint,
                              const alice::SightMessage* message, bool use_compression);
  // Writes a channel statistics report to the console
  void writeChannelStatisticsToConsole();

  alice::MessageLedger* message_ledger_;
  alice::SightChannelStatus* channel_status_component_;

  std::map<uWS::WebSocket<uWS::SERVER>*, Connection> socket_servers_;
  std::mutex mutex_;
  std::set<std::string> tags_;
  std::unique_ptr<uWS::Hub> hub_;

  std::map<std::string, std::string> channel_uuid_;

  std::mutex resources_mutex_;
  std::map<std::string, std::vector<char>> resources_;
  std::map<std::string, std::vector<char>> compressed_resources_;

  std::unordered_map<std::string, ChannelStatistics> channel_statistics_;
};

}  // namespace sight
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::sight::WebsightServer);
