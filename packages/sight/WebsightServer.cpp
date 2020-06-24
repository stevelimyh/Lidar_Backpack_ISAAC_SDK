/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "WebsightServer.hpp"

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/backend/sight_backend.hpp"
#include "engine/alice/components/SightChannelStatus.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/constants.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "engine/gems/serialization/files.hpp"
#include "engine/gems/serialization/json_formatter.hpp"
#include "engine/gems/sight/sight.hpp"
#include "packages/sight/SightWidget.hpp"

namespace isaac {
namespace sight {

namespace {

// The maximum size of an incoming message we accept on websockets. This is a security measure.
constexpr size_t kMaxIncomingMessageSize = 64'000'000;
// The current version of Websight
constexpr char kSightVersion[] = "1.2";
// The header key for encoding
constexpr char kKeyEncoding[] = "accept-encoding";
// The compression format supported by WebsightServer
constexpr char kCompressionFormat[] = "deflate";

// Helper class to check if the queue is empty:
// WebSocket inherit from Socket which does not expose any way to get the status of the queue.
// However the function hasEmptyQueue is protected and can be accessed by inheriting from WebSocket.
class WebSocketHelper : public uWS::WebSocket<uWS::SERVER> {
 public:
  // This class should not be instantiated directly.
  WebSocketHelper() = delete;
  // Returns whether or not the queue is empty.
  bool hasEmptyQueue() {
    return this->uWS::WebSocket<uWS::SERVER>::hasEmptyQueue();
  }
};

}  // namespace

Json WebsightServer::getAllTagsMessage(uWS::WebSocket<uWS::SERVER>* ws) {
  Json json_out;
  json_out["type"] = "getAllTags";
  Json json_data;
  for (auto tag : tags_) {
    Json data;
    data["uuid"] = channel_uuid_[tag];
    data["name"] = tag;
    data["checked"] = socket_servers_[ws].enabled_tags.count(tag) > 0;
    json_data.push_back(data);
  }
  json_out["data"] = json_data;
  return json_out;
}

void WebsightServer::setTagsStatusHandle(const Json& json_in, uWS::WebSocket<uWS::SERVER>* ws) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::set<std::string> tags;
  std::string value = json_in["value"].get<std::string>();
  const bool enabled = json_in["enable"].get<bool>();
  if (enabled) {
    if (socket_servers_[ws].enabled_tags.count(value) == 0) {
      socket_servers_[ws].enabled_tags.emplace(value);
      channel_status_component_->addChannelListener(value);
    }
  } else {
    if (socket_servers_[ws].enabled_tags.count(value) > 0) {
      socket_servers_[ws].enabled_tags.erase(value);
      channel_status_component_->removeChannelListener(value);
    }
  }
}

void WebsightServer::start() {
  tickBlocking();

  message_ledger_ = node()->getComponent<alice::MessageLedger>();

  hub_.reset(new uWS::Hub());
  hub_->onConnection([this](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Disable all tags by default
    socket_servers_[ws].enabled_tags = {};
    socket_servers_[ws].flow_control.resetTargetBandwith(get_bandwidth());
    LOG_INFO("Server connected %s %d", req.getUrl().toString().c_str(), socket_servers_.size());
  });

  hub_->onDisconnection([this](uWS::WebSocket<uWS::SERVER> *ws,  int code, char *message,
                               size_t length) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = socket_servers_.find(ws);
    for (const auto& tag : it->second.enabled_tags) {
      channel_status_component_->removeChannelListener(tag);
    }
    socket_servers_.erase(it);
    LOG_INFO("Disconnected");
  });

  // Callback to handle incoming messages
  hub_->onMessage([this](uWS::WebSocket<uWS::SERVER>* ws, char* message, size_t length,
                      uWS::OpCode opCode) {
    // Check that the message is not too big
    if (length > kMaxIncomingMessageSize) {
      LOG_ERROR("Incoming message is too big: %zd", length);
      return;
    }
    // Try to parse the message as JSON
    const std::string json_text(message, length);
    auto maybe_json_in = serialization::ParseJson(json_text);
    if (!maybe_json_in) {
      LOG_ERROR("Incoming message must be a JSON object.");
      return;
    }
    // Parse the received JSON message
    // We expect the following format: { "type": type, "data": data }
    std::string message_type;
    const auto maybe_type = serialization::TryGetFromMap<std::string>(*maybe_json_in, "type");
    if (!maybe_type) {
      LOG_ERROR("Incoming JSON message does not have an element 'type'");
      return;
    } else {
      message_type = *maybe_type;
    }
    Json* maybe_data = nullptr;
    const auto it = maybe_json_in->find("data");
    if (it != maybe_json_in->end()) {
      maybe_data = &(*it);
    }
    // React to the incoming message
    Json json_out;
    if (message_type == "getAllTags") {
      std::lock_guard<std::mutex> lock(mutex_);
      json_out = getAllTagsMessage(ws);
    } else if (message_type == "load_ui_config") {
      json_out = getUiConfigJson();
    } else if (message_type == "getLatest") {
      std::lock_guard<std::mutex> lock(mutex_);
      json_out = getLatestMessages(ws);
    } else if (message_type == "getVersion") {
      json_out["type"] = "getVersion";
      json_out["version"] = kSightVersion;
    } else if (message_type == "setTagsStatus") {
      if (!maybe_data) {
        LOG_ERROR("Incoming JSON message does not have an element 'data': %s", json_text.c_str());
        return;
      }
      setTagsStatusHandle(*maybe_data, ws);
      return;
    } else {
      if (!maybe_data) {
        LOG_ERROR("Incoming JSON message does not have an element 'data': %s", json_text.c_str());
        return;
      }
      // This is the new standard path
      // TODO Remove the other paths above
      auto message = std::make_shared<alice::JsonMessage>();
      message->uuid = Uuid::Generate();
      message->pubtime = node()->clock()->timestamp();
      message->acqtime = message->acqtime;
      message->data = std::move(*maybe_data);
      message_ledger_->provide({this, message_type}, message);
      return;
    }
    const std::string message_out = json_out.dump();
    ws->send(message_out.data(), message_out.length(), opCode);
  });

  hub_->onHttpRequest([&](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t length,
                          size_t remainingBytes) {
    bool use_compression = false;
    // If compression is request, check the browser support the compression format.
    if (get_use_compression()) {
      auto header = req.getHeader(kKeyEncoding, 15);
      if (header && header.toString().find(kCompressionFormat)) {
        use_compression = true;
      }
    }
    const auto url = getResourceFilename(req.getUrl().toString());
    const std::vector<char>& buffer =
        use_compression ? loadCompressedResource(url, use_compression) : loadResource(url);
    res->end(buffer.data(), buffer.size(), nullptr, nullptr, use_compression);
  });

  if (!hub_->listen(get_port())) {
    LOG_ERROR("Failed to start Webserver on port %d!\nMake sure no other app is running and that "
              "the previous app has been closed properly", get_port());
  } else {
    LOG_INFO("Sight webserver is loaded");
    LOG_INFO("Please open Chrome Browser and navigate to http://<ip address>:%d", get_port());
  }

  node()->app()->backend()->sight_backend()->registerCustomer(message_ledger_);

  channel_status_component_ = node()->getComponent<alice::SightChannelStatus>();
  node()->app()->backend()->sight_backend()->registerChannelStatus(channel_status_component_);
  tags_ = channel_status_component_->getListChannels();
}

void WebsightServer::addTags(const std::string& name) {
  if (tags_.count(name)) return;
  tags_.insert(name);
  channel_uuid_[name] = std::to_string(channel_uuid_.size());

  for (auto& client : socket_servers_) {
    client.second.enabled_tags.insert(name);
    channel_status_component_->addChannelListener(name);
    Json json_out = getAllTagsMessage(client.first);
    const std::string message_out = json_out.dump();
    client.first->send(message_out.data(), message_out.length(), uWS::OpCode::TEXT);
  }
}

Json WebsightServer::getLatestMessages(uWS::WebSocket<uWS::SERVER>* ws) {
  Json json_out;
  json_out["type"] = "getLatest";
  Json json_data;
  message_ledger_->readAllLatest(
      [this, &json_data](const alice::MessageLedger::Endpoint& endpoint,
                         const alice::ConstMessageBasePtr& message) {
    // Only send messages which come from the outside
    if (endpoint.component == this) {
      return;
    }
    // Currently we need to handle JsonMessage and SightMessage
    auto json_message = std::dynamic_pointer_cast<const alice::JsonMessage>(message);
    if (json_message) {
      return;
    }
    const std::string endpoint_name = endpoint.nameWithApp();
    addTags(endpoint_name);
    auto sight_message = std::dynamic_pointer_cast<const alice::SightMessage>(message);
    ASSERT(sight_message, "Not of type SightMessage");
    std::string json_str;
    auto json = sight_message->json;
    json["uuid"] = channel_uuid_[endpoint_name];
    json["name"] = endpoint_name;
    json_data.push_back(json);
  });
  json_out["data"] = std::move(json_data);
  return json_out;
}

void WebsightServer::tick() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // send all new messages
    const bool use_compression = get_use_compression();
    message_ledger_->readAllNew(
        [this, use_compression](const alice::MessageLedger::Endpoint& endpoint,
                                const alice::ConstMessageBasePtr& message) {
      // Currently we can to handle JsonMessage and SightMessage
      const auto* json_message = dynamic_cast<const alice::JsonMessage*>(message.get());
      const auto* sight_message = dynamic_cast<const alice::SightMessage*>(message.get());
      if (json_message) {
        processNewJsonMessage(endpoint, json_message);
      } else if (sight_message) {
        processNewSightMessage(endpoint, sight_message, use_compression);
      } else {
        LOG_ERROR("Unknown message type. Can only handle JsonMessage and SightMessage.");
      }
    });
  }
  hub_->getLoop()->doEpoll(5);
}

void WebsightServer::stop() {
  dynamic_cast<uWS::Group<uWS::SERVER>*>(hub_.get())->close();
  hub_.reset();
  writeChannelStatisticsToConsole();
}

std::string WebsightServer::getResourceFilename(const std::string& requested_url) {
  const std::string asset_prefix = "/apps/assets/";
  const std::string webroot = node()->app()->getAssetPath(get_webroot());
  const std::string assetroot = node()->app()->getAssetPath(get_assetroot());
  if (requested_url == "/") {
    // the default is the index page
    return webroot + "/index.html";
  } else if (requested_url.substr(0, asset_prefix.size()) == asset_prefix) {
    // If the requested URL starts with '/apps/assets/' we are loading an asset
    return assetroot + "/" + requested_url.substr(asset_prefix.size());
  } else {
    // otherwise we assume that a web ressouce is requested
    return webroot + "/" + requested_url;
  }
}

const std::vector<char>& WebsightServer::loadCompressedResource(const std::string& filename,
                                                                bool& compression_used) {
  { // Check if the rousource is cached
    std::lock_guard<std::mutex> lock(resources_mutex_);
    auto it = compressed_resources_.find(filename);
    if (it != compressed_resources_.end()) {
      return it->second;
    }
  }
  // Unlock while getting the uncompressed version
  const std::vector<char>& buffer = loadResource(filename);
  size_t size = buffer.size();
  // This should be enough memory, if not we are better of sending the uncompressed version.
  std::vector<char> buffer_compressed(size);
  if (compress(reinterpret_cast<uint8_t*>(buffer_compressed.data()), &size,
               reinterpret_cast<const uint8_t*>(buffer.data()), size) != Z_OK) {
    compression_used = false;
    return buffer;
  }
  // Resize the buffer to match exactly the size.
  buffer_compressed.resize(size);
  buffer_compressed.shrink_to_fit();
  compression_used = true;
  std::lock_guard<std::mutex> lock(resources_mutex_);
  auto it = compressed_resources_.insert({filename, std::move(buffer_compressed)}).first;
  return it->second;
}

const std::vector<char>& WebsightServer::loadResource(const std::string& filename) {
  { // Check if the rousource is cached
    std::lock_guard<std::mutex> lock(resources_mutex_);
    auto it = resources_.find(filename);
    if (it != resources_.end()) {
      return it->second;
    }
  }
  // Unlock while reading the file on disk.
  // TODO Maybe load some files in text mode instead?
  std::vector<char> buffer;
  if (!serialization::ReadEntireBinaryFile(filename, buffer)) {
    LOG_ERROR("Error loading websight resource from file '%s'", filename.c_str());
  }
  std::lock_guard<std::mutex> lock(resources_mutex_);
  auto it = resources_.insert({filename, std::move(buffer)}).first;
  return it->second;
}

void WebsightServer::sendJson(const Json& json, uWS::WebSocket<uWS::SERVER>* ws) const {
  const std::string json_str = json.dump();
  ws->send(json_str.c_str(), json_str.length(), uWS::OpCode::TEXT);
}

Json WebsightServer::getUiConfigJson() const {
  Json json;
  json["type"] = "load_ui_config";
  json["app_name"] = node()->app()->name();

  json["config"] = get_ui_config();
  for (auto* widget : node()->app()->findComponents<SightWidget>()) {
    if (const auto maybe_specs = widget->toJson()) {
      json["config"]["windows"][maybe_specs->first] = std::move(maybe_specs->second);
    }
  }

  return json;
}

bool WebsightServer::isReplyEndpoint(const alice::MessageLedger::Endpoint& endpoint) const {
  return endpoint.component == this && EndsWith(endpoint.tag, "_reply");
}

void WebsightServer::processNewJsonMessage(
    const alice::MessageLedger::Endpoint& endpoint, const alice::JsonMessage* message) {
  // Only send messages which come from the outside
  if (!isReplyEndpoint(endpoint)) {
    return;
  }
  Json reply;
  reply["type"] = endpoint.tag;
  reply["data"] = message->data;
  const std::string message_out = reply.dump();
  for (auto& socket_pair : socket_servers_) {
    socket_pair.first->send(message_out.data(), message_out.length(), uWS::OpCode::TEXT);
  }
}

void WebsightServer::processNewSightMessage(
    const alice::MessageLedger::Endpoint& endpoint, const alice::SightMessage* message,
    bool use_compression) {
  // send show data to frontend
  const std::string endpoint_name = endpoint.nameWithApp();
  addTags(endpoint_name);
  std::string json_str;
  // TODO(cdelaunay) Right now we have one frequency for all the clients
  for (auto& socket_pair : socket_servers_) {
    if (!static_cast<WebSocketHelper*>(socket_pair.first)->hasEmptyQueue()) {
      continue;
    }
    if (socket_pair.second.enabled_tags.count(endpoint_name) == 0) {
      continue;
    }
    if (json_str.empty()) {
      auto json = message->json;
      json["uuid"] = channel_uuid_[endpoint_name];
      json_str = json.dump();
    }
    if (!socket_pair.second.flow_control.keepMessage(endpoint_name, NowCount(),
                                                     json_str.size())) {
      continue;
    }
    socket_pair.first->send(json_str.c_str(), json_str.length(), uWS::OpCode::TEXT,
                            nullptr, nullptr, use_compression);
  }
  // Update statistics
  if (!json_str.empty()) {
    const double total_mb = static_cast<double>(json_str.size()) / 1'048'576.0;
    auto& statistics = channel_statistics_[endpoint.name()];
    statistics.current_bandwidth.add(total_mb, getTickTime());
    statistics.total += total_mb;
    statistics.count++;
  }
}

void WebsightServer::writeChannelStatisticsToConsole() {
  constexpr size_t kNumItems = 15;
  // First create a copy
  std::vector<std::pair<std::string, ChannelStatistics>> stats;
  stats.insert(stats.end(), channel_statistics_.begin(), channel_statistics_.end());
  // Sort by total (biggest first)
  std::sort(stats.begin(), stats.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.second.total > rhs.second.total;
      });

  LOG_INFO("=====================================================================================");
  LOG_INFO("|                             Websight Statistics Report                            |");
  LOG_INFO("=====================================================================================");
  LOG_INFO("| %-47.47s | %8s | %8s | %8s |", "Name", "Bandwidth", "Total", "Count");
  LOG_INFO("-------------------------------------------------------------------------------------");
  for (size_t i = 0; i < std::min(stats.size(), kNumItems); i++) {
    const auto& kvp = stats[i];
    LOG_INFO("| %-47.47s | %9.2f | %8.2f | %8d |", TakeLast(kvp.first, 47).c_str(),
             kvp.second.current_bandwidth.rate(), kvp.second.total, kvp.second.count);
  }
  if (stats.size() > kNumItems) {
    double other_bandwidth = 0.0;
    double other_total = 0.0;
    size_t other_count = 0;
    for (size_t i = kNumItems; i < stats.size(); i++) {
      const auto& kvp = stats[i];
      other_bandwidth += kvp.second.current_bandwidth.rate();
      other_total += kvp.second.total;
      other_count += kvp.second.count;
    }
    LOG_INFO("| %8zd %-38.38s | %9.2f | %8.2f | %8d |", stats.size() - kNumItems, "more",
             other_bandwidth, other_total, other_count);
  }
  LOG_INFO("=====================================================================================");
}

}  // namespace sight
}  // namespace isaac
