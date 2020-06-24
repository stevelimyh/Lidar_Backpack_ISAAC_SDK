/*
 Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

 NVIDIA CORPORATION and its licensors retain all intellectual property
 and proprietary rights in and to this software, related documentation
 and any modifications thereto. Any use, reproduction, disclosure or
 distribution of this software and related documentation without an express
 license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#include "LivoxLidar.hpp"

#include <endian.h>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/constants.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/coms/socket.hpp"
#include "engine/gems/image/color.hpp"
#include "messages/math.hpp"
#include "messages/tensor.hpp"
#include "sdk_core/include/livox_def.h"
#include "sdk_core/src/comm/sdk_protocol.h"

namespace {

// Livox placed these value in a source file, although we need them.
constexpr uint8_t kSdkProtocolSof = 0xAA;        // Livox start-of-frame (magic) number.
constexpr size_t kSdkPacketCrcSize = 4;          // CRC32 size.
constexpr size_t kSdkPacketPreambleCrcSize = 2;  // CRC16 size.

// Each data sample from the lidar contains one hundred data point.
constexpr size_t kPointSampleSize = 100;

// Requests sent and responses received are expected to meet protocol payload sizes (in bytes).
constexpr size_t kHeartbeatRequestPayloadSize = 15;
constexpr size_t kHeartbeatResponsePayloadSize = 22;
constexpr size_t kHandshakeRequestPayloadSize = 23;
constexpr size_t kHandshakeResponsePayloadSize = 16;
constexpr size_t kSamplingRequestPayloadSize = 16;
constexpr size_t kSamplingResponsePayloadSize = 16;
constexpr size_t kCoodinateSystemRequestPayloadSize = 16;
constexpr size_t kCoodinateSystemResponsePayloadSize = 16;
constexpr size_t kDataSamplePayloadSize = 1318;
constexpr size_t kCoordinateSystemCartesian = 0;

// The UDP port to handshake with the lidar and configure the command and data port. The port
// 65000 is a Livox design decision of their communication protocol.
constexpr uint16_t kLivoxLidarHandshakePort = 65000;

}  // namespace

namespace isaac {

void LivoxLidar::start() {
  // We validate the lidar's configuration data contracts against common pitfalls.
  const std::string device_ip(get_device_ip());
  struct sockaddr_in socket_address;
  if (!device_ip.compare("0.0.0.0") ||
      inet_pton(AF_INET, device_ip.c_str(), &(socket_address.sin_addr)) <= 0) {
    reportFailure("Failed to configure Livox lidar IPv4 address: configured address = %s",
                  device_ip.c_str());
    return;
  }

  const int port_command = get_port_command();
  if ((port_command < 0 || port_command > std::numeric_limits<uint16_t>::max()) &&
      port_command != kLivoxLidarHandshakePort) {
    reportFailure("Failed to configure Livox lidar UDP command port: configured port = %d",
                  port_command);
    return;
  }

  const int port_data = get_port_data();
  if ((port_data < 0 || port_data > std::numeric_limits<uint16_t>::max()) &&
      port_data != kLivoxLidarHandshakePort && port_data != port_command) {
    reportFailure("Failed to configure Livox lidar UDP data port: configured port = %d", port_data);
    return;
  }

  const int batch_count = get_batch_count();
  if (batch_count <= 0) {
    reportFailure("Failed to configure Livox lidar point batch count: count = %d", batch_count);
    return;
  }
  expected_batch_count_ = batch_count;

  // We reset/open the UDP sockets for handshake, command, and data communication channels.
  socket_handshake_.reset(Socket::CreateRxUDPSocket(device_ip, kLivoxLidarHandshakePort));
  size_t socket_response_code_or_length = socket_handshake_->startSocket();
  if (socket_response_code_or_length < 0) {
    reportFailure(
        "Failed to reset UDP socket for Livox lidar handshake: port = %d, code = %d, errno = %d",
        kLivoxLidarHandshakePort, socket_response_code_or_length, errno);
    return;
  }

  socket_command_.reset(Socket::CreateRxUDPSocket(device_ip, port_command));
  socket_response_code_or_length = socket_command_->startSocket();
  if (socket_response_code_or_length < 0) {
    reportFailure(
        "Failed to reset UDP socket for Livox lidar commands: port = %d, code = %d, errno = %d",
        port_command, socket_response_code_or_length, errno);
    return;
  }

  socket_data_.reset(Socket::CreateRxUDPSocket(device_ip, port_data));
  socket_response_code_or_length = socket_data_->startSocket();
  if (socket_response_code_or_length < 0) {
    reportFailure(
        "Failed to reset UDP socket for Livox lidar data: port = %d, code = %d, errno = %d",
        port_data, socket_response_code_or_length, errno);
    return;
  }

  // Raw UDP datagram/packet buffer, size is defined per Livox communication protocol. We must
  // guarantee the size of the container with valid elements for the socket API.
  raw_packet_buffer_.resize(livox::kMaxCommandBufferSize);

  // We seed/reset the cached message acqtime on start/restart.
  previous_message_sent_acq_time_ = 0;

  // Livox command frame sequence number starts/restarts at 0 for simplicity.
  sequence_command_ = 0;

  // Seed the CRC16 and CRC32 with the corresponding Livox seeds for preamble and frame error
  // detection.
  crc16_ = 0x4c49;
  crc32_ = 0x564f580a;

  // We seed/reset the clock synchronization offset on start/restart, along with the heartbeat
  // related primitives.
  driver_acqtime_offset_ = 0;
  last_sent_hearbeat_time_ = 0.0;
  hearbeat_response_pending_ = false;

  // A small initialization sequence and handshake is needed to established communication
  // with the device. First the handshake, then we start the data sampling.
  socket_response_code_or_length = writeLidarHandshakeRequest();
  if (socket_response_code_or_length != kHandshakeRequestPayloadSize) {
    reportFailure("Failed to send handshake request for Livox lidar: code = %d, errno = %d",
                  socket_response_code_or_length, errno);
    return;
  }

  socket_response_code_or_length =
      socket_command_->readPacket((raw_packet_buffer_.data()), raw_packet_buffer_.size() - 1);
  if (socket_response_code_or_length != kHandshakeResponsePayloadSize) {
    reportFailure("Failed to receive handshake response for Livox lidar: code = %d, errno = %d",
                  socket_response_code_or_length, errno);
    return;
  }

  socket_response_code_or_length = writeLidarChangeCoordinateSystemRequest();
  if (socket_response_code_or_length != kCoodinateSystemRequestPayloadSize) {
    reportFailure(
        "Failed to send change coordinate system request for Livox lidar: code = %d, errno = %d",
        socket_response_code_or_length, errno);
    return;
  }

  socket_response_code_or_length =
      socket_command_->readPacket((raw_packet_buffer_.data()), raw_packet_buffer_.size() - 1);
  if (socket_response_code_or_length != kCoodinateSystemResponsePayloadSize) {
    reportFailure(
        "Failed to receive change coordinate system response for Livox lidar: code = %d, errno = "
        "%d",
        socket_response_code_or_length, errno);
    return;
  }

  socket_response_code_or_length = writeLidarStartSamplingRequest();
  if (socket_response_code_or_length != kSamplingRequestPayloadSize) {
    reportFailure("Failed to send start sampling request for Livox lidar: code = %d, errno = %d",
                  socket_response_code_or_length, errno);
    return;
  }

  socket_response_code_or_length =
      socket_command_->readPacket((raw_packet_buffer_.data()), raw_packet_buffer_.size() - 1);
  if (socket_response_code_or_length != kSamplingResponsePayloadSize) {
    reportFailure("Failed to receive sampling response for Livox lidar: code = %d, errno = %d",
                  socket_response_code_or_length, errno);
    return;
  }

  allocate_data_ = true;
  current_batch_position_ = 0;

  tickBlocking();
}

void LivoxLidar::stop() {
  socket_handshake_->closeSocket();
  socket_command_->closeSocket();
  socket_data_->closeSocket();
}

void LivoxLidar::tick() {
  // The primary goal of the driver is to allow data to flow into the system. This is our most
  // important activity. Upon the initialization sequence completed, the lidar would start sending
  // data on the UDP socket for data.
  size_t socket_response_code_or_length =
      socket_data_->readPacket((raw_packet_buffer_.data()), raw_packet_buffer_.size() - 1);

  if (socket_response_code_or_length != kDataSamplePayloadSize) {
    reportFailure("Unexpected lidar data sample size received: size = %d, errno = %d",
                  socket_response_code_or_length, errno);
    return;
  } else {
    // We use the node time as acqtime. When compared to the absolute data sampling time, the node
    // time is subjected to network and lidar internal latencies. While this introduces variable
    // jitter and skew, it is a much simpler strategy to handle publishing acqtime. We provide
    // additional metrics below to show latencies and RTC differences. All timestamps are in
    // nanoseconds. We use the maximum/last node timestamp as the acquisition time. This is the
    // equivalent behavior from the scan accumulator component however one could question whether
    // we should use the minimum, average, or hardware related form of acquisition time.
    const int64_t acqtime = node()->clock()->timestamp();

    // We prevent invalid runtime configuration of the batch count and race condition in the
    // batch size transmitted.
    const int batch_count = get_batch_count();
    if (batch_count <= 0) {
      reportFailure("Failed to configure Livox lidar point batch count at runtime: count = %d",
                    batch_count);
      return;
    }

    // We interpret the data received from the device and we append this data to the point cloud
    // proto we are populating.
    LivoxRawPoint* livox_data_samples = reinterpret_cast<LivoxRawPoint*>(
        (reinterpret_cast<LivoxEthPacket*>(raw_packet_buffer_.data()))->data);
    // Initialize the builder and its components we use to accumulate data over ticks.
    if (allocate_data_) {
      positions_ = SampleCloud3f(expected_batch_count_ * kPointSampleSize);
      intensities_ = SampleCloud1f(expected_batch_count_ * kPointSampleSize);
      current_batch_position_ = 0;
      allocate_data_ = false;
    }
    constexpr float kMillimeterToMeterRatio = 1.0e+3f;
    for (size_t data_index = 0; data_index < kPointSampleSize; data_index++) {
      positions_(data_index + current_batch_position_ * kPointSampleSize) =
          Vector3f(static_cast<float>(livox_data_samples[data_index].x) / kMillimeterToMeterRatio,
                   static_cast<float>(livox_data_samples[data_index].y) / kMillimeterToMeterRatio,
                   static_cast<float>(livox_data_samples[data_index].z) / kMillimeterToMeterRatio);
      intensities_(data_index + current_batch_position_ * kPointSampleSize) =
          MapUint8ToUnit(livox_data_samples[data_index].reflectivity);

      // We could support a delta time between points: 10 microseconds.
    }
    current_batch_position_++;

    if (current_batch_position_ == expected_batch_count_) {
      auto proto = tx_accumulated_point_cloud().initProto();
      proto.initPositions();
      proto.initIntensities();
      ToProto(std::move(positions_), proto.getPositions(), tx_accumulated_point_cloud().buffers());
      ToProto(std::move(intensities_), proto.getIntensities(),
              tx_accumulated_point_cloud().buffers());

      tx_accumulated_point_cloud().publish(acqtime);
      allocate_data_ = true;

      // We report the number of points per message sent. Note that we do not report the number
      // of point received from the Livox lidar because it is stricly constant at 100 points per
      // incoming message, by protocol design.
      show("points_per_message_sent", expected_batch_count_ * kPointSampleSize);

      // We provide metrics about message sent rates and we care not to assume all platforms support
      // IEEE-754/IEC-559 so we explicitly handle division by zero. We then reset the tracked time.
      // Note this is not an averaged rate.
      double seconds_since_last_publishing =
          ToSeconds(std::fabs(acqtime - previous_message_sent_acq_time_));
      if (IsAlmostZero(seconds_since_last_publishing)) {
        seconds_since_last_publishing = MachineEpsilon<double>;
      }
      const double messages_sent_per_second = 1.0 / seconds_since_last_publishing;
      show("messages_sent_per_second", messages_sent_per_second);
      previous_message_sent_acq_time_ = acqtime;

      // Lastly we clean up the buffer and update the expected batch count.
      // We only allow changes to the batch count on message sending to prevent race condition
      // in the expected versus actual message size (including missing data or message overflow).
      // We are ready for the next points.
      expected_batch_count_ = batch_count;
    } else if (current_batch_position_ > expected_batch_count_) {
      // We should never enter this defensive block. If we, we are likely to have a memory
      // corruption. Most likely due to a violation in the batch count input contract or its
      // accumulation.
      ASSERT(false, "Livox lidar point batch count overflow: %d, expected: %d",
             current_batch_position_, expected_batch_count_);
      return;
    }

    // We provide metrics about time clock differences between the platform running Isaac and the
    // embedded device lidar. We can show the clock differences between the device and the system by
    // accounting for network and device latencies. This allows us to visualize the clocks jitter
    // and skew. The clock drift was measured experimentally on one unit and found to be around two
    // seconds per day. We seed the network data offset time once, when we are about to publish our
    // first scan, this aligns well with using the maximum/last node time or even the data sample
    // time.
    const int64_t data_time = *reinterpret_cast<uint64_t*>(
        &(reinterpret_cast<LivoxEthPacket*>(raw_packet_buffer_.data()))->timestamp);
    if (!driver_acqtime_offset_.value_or(0)) {
      driver_acqtime_offset_ = acqtime - data_time;
    }
    const int64_t sync_time = data_time + driver_acqtime_offset_.value();
    const int64_t clocks_difference_nanoseconds = acqtime - sync_time;
    show("clocks_difference_nanoseconds", clocks_difference_nanoseconds);

    // A one-second periodic heartbeat is used as an application level keep-alive with the device.
    // As we use a blocking UDP socket (with a configured one second timeout), we interleave the
    // heartbeat request sending and response processing with the data. We do this for performance
    // reasons: we don't want to block while we wait for the heartbeat round trip which was
    // measured to be in the best case 3 ms, and where the worst case would depend on network
    // conditions. We chose a reasonable 200 ms timeout.
    constexpr double kHeartbeatPeriod = 1.0;
    constexpr double kHeartbeatTimeout = 0.2;
    constexpr double kHeartbeatSendTime = kHeartbeatPeriod - kHeartbeatTimeout;

    const double current_tick_time = getTickTime();

    if (current_tick_time - last_sent_hearbeat_time_ > kHeartbeatSendTime) {
      size_t socket_response_code_or_length = writeLidarHeartbeatRequest();
      if (socket_response_code_or_length != kHeartbeatRequestPayloadSize) {
        reportFailure("Failed to send heartbeat request for Livox lidar: code = %d, errno = %d",
                      socket_response_code_or_length, errno);
        return;
      }
      last_sent_hearbeat_time_ = current_tick_time;
      hearbeat_response_pending_ = true;
    }

    if (hearbeat_response_pending_ &&
        (current_tick_time - last_sent_hearbeat_time_ > kHeartbeatTimeout)) {
      size_t socket_response_code_or_length =
          socket_command_->readPacket((raw_packet_buffer_.data()), raw_packet_buffer_.size() - 1);
      if (socket_response_code_or_length != kHeartbeatResponsePayloadSize) {
        reportFailure("Failed to receive heartbeat response for Livox lidar: code = %d, errno = %d",
                      socket_response_code_or_length, errno);
        return;
      }

      hearbeat_response_pending_ = false;
    }
  }
}  // namespace isaac

size_t LivoxLidar::writeLidarHandshakeRequest() {
  // The Livox handshake payload data is populated.
  HandshakeRequest payload;
  uint32_t local_ip = socket_handshake_->getIpv4();
  payload.ip_addr = local_ip;
  payload.cmd_port = get_port_command();
  payload.data_port = get_port_data();

  return writeLidarRequest(socket_handshake_, reinterpret_cast<uint8_t*>(&payload), sizeof(payload),
                           livox::kCommandIDGeneralHandshake);
}

size_t LivoxLidar::writeLidarHeartbeatRequest() {
  // There is no payload for heartbeat commands.
  return writeLidarRequest(socket_command_, nullptr, 0, livox::kCommandIDGeneralHeartbeat);
}

size_t LivoxLidar::writeLidarStartSamplingRequest() {
  // The payload of the start/stop sampling command consists of a single boolean.
  bool start_sampling = true;
  uint8_t payload = start_sampling;

  return writeLidarRequest(socket_command_, &payload, sizeof(payload),
                           livox::kCommandIDGeneralControlSample);
}

size_t LivoxLidar::writeLidarChangeCoordinateSystemRequest() {
  // The payload of the change coordinate system command consists of a single uint8_t value.
  uint8_t payload = kCoordinateSystemCartesian;

  return writeLidarRequest(socket_command_, &payload, sizeof(payload),
                           livox::kCommandIDGeneralCoordinateSystem);
}

size_t LivoxLidar::writeLidarRequest(std::unique_ptr<Socket>& socket, uint8_t* payload,
                                          uint16_t payload_size,
                                          livox::GeneralCommandID command_code) {
  // The raw UDP frame is created, populated from the Livox frame and the handshake payload.
  // The data is packed, the checksums computed, the size finalized.
  uint8_t local_raw_packet_buffer[livox::kMaxCommandBufferSize + 1]{0};
  livox::SdkPacket* sdk_packet = reinterpret_cast<livox::SdkPacket*>(local_raw_packet_buffer);
  const uint16_t packet_length = payload_size + sizeof(livox::SdkPacket) - 1 + kSdkPacketCrcSize;
  size_t socket_response_code_or_length = -1;

  if (packet_length <= livox::kMaxCommandBufferSize) {
    sdk_packet->length = packet_length;
    sdk_packet->sof = kSdkProtocolSof;
    sdk_packet->version = livox::kSdkVer0;
    sdk_packet->packet_type = livox::kCommandTypeCmd;
    sdk_packet->seq_num = sequence_command_++;
    sdk_packet->preamble_crc = crc16_->mcrf4xx_calc(
        local_raw_packet_buffer, sizeof(livox::SdkPreamble) - kSdkPacketPreambleCrcSize);
    sdk_packet->cmd_set = livox::kCommandSetGeneral;
    sdk_packet->cmd_id = command_code;
    std::memcpy(sdk_packet->data, payload, payload_size);

    // Lastly, we compute the final checksum and place it in the trailing four bytes of the packet
    // before eventually sending the data.
    uint32_t crc = crc32_->crc32_calc(local_raw_packet_buffer, packet_length - kSdkPacketCrcSize);
    *reinterpret_cast<uint32_t*>(&local_raw_packet_buffer[packet_length - 4]) = crc;

    socket_response_code_or_length =
        socket->writePacket(local_raw_packet_buffer, sdk_packet->length);
  } else {
    LOG_ERROR("Packet length unexpectedly large (%d).", packet_length);
  }

  return socket_response_code_or_length;
}

}  // namespace isaac
