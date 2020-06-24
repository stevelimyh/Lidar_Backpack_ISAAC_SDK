/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/tensor/sample_cloud.hpp"
#include "engine/gems/serialization/json.hpp"
#include "messages/point_cloud.capnp.h"
#include "sdk_core/include/third_party/FastCRC/FastCRC.h"
#include "sdk_core/src/command_handler/command_impl.h"

namespace isaac {

class Socket;

// A driver for the Livox Mid-40 lidar. The driver opens and maintains the UDP sockets for
// communication with the lidar. The driver accumulates samples and publish them as a range point
// cloud. A missing lidar or incorrect configuration results in the component stopping. The lidar
// publishes 100,000 points per second in size-configurable batches. A drop in communication (e.g.
// loose cable, network interface change) will result in the component stopping.
class LivoxLidar : public alice::Codelet {
 public:
  // Standard codelet/component start function override.
  void start() override;
  // Standard codelet/component tick function override. Block on receiving data from the lidar and
  // sends the periodic lidar heartbeat.
  void tick() override;
  // Standard codelet/component stop function override. Perform the activities necessary to
  // gracefully stop using the lidar.
  void stop() override;

  // Output 3D point cloud samples. The point cloud is published when the point count reaches the
  // configured minimum point count or when the time between message publishing is greater than the
  // configured published interval.
  ISAAC_PROTO_TX(PointCloudProto, accumulated_point_cloud);

  // The IP address of the lidar device we want to connect to and receive data from. This parameter
  // is changeable at configuration time.
  ISAAC_PARAM(std::string, device_ip, "0.0.0.0");
  // The UDP port to send commands to the lidar. This parameter is changeable at configuration time.
  ISAAC_PARAM(int, port_command, 50001);
  // The UDP port from which the data samples will be received. This parameter is changeable at
  // configuration time.
  ISAAC_PARAM(int, port_data, 50002);
  // Minimum number of accumulated point batches before publishing the point cloud. It can be
  // configured and changed at runtime. The point cloud is published when the point count reaches
  // the configured point batch count. Each batch is 100 data points per Livox communication
  // protocol.
  ISAAC_PARAM(int, batch_count, 10);

 private:
  // Perform the activities necessary to receive data from the lidar: open UDP sockets, handshake,
  // configure, retrieve configuration, and start data sampling. Also support restarting lidar
  // activities for recovery.
  bool startLidar();

  // Build the payload for a Livox lidar handshake request, build, and send the full frame. The
  // return value is the socket response code or the length per the typical socket API.
  size_t writeLidarHandshakeRequest();
  // Build the payload (or lack thereof) for a Livox lidar handshake request, build, and send the
  // full frame. Convenient. The return value is the socket response code or the length per the
  // typical socket API.
  size_t writeLidarHeartbeatRequest();
  // Build the payload for a Livox lidar start sampling request, build, and send the full frame. The
  // return value is the socket response code or the length per the typical socket API.
  size_t writeLidarStartSamplingRequest();
  // Build the payload for a Livox lidar coordinate system change request, build, and send the full
  // frame. The return value is the socket response code or the length per the typical socket API.
  size_t writeLidarChangeCoordinateSystemRequest();

  // Build the Livox frame with the provided payload. Add all expected data fields according to
  // Livox's communication protocol including the CRC. The return value is the socket response code
  // or the length per the typical socket API.
  size_t writeLidarRequest(std::unique_ptr<Socket>& socket, uint8_t* payload,
                                uint16_t payload_size, livox::GeneralCommandID command_code);

  // The bidirectional communication socket for the handshake with the device (UDP).
  std::unique_ptr<Socket> socket_handshake_;
  // The bidirectional communication socket for sending commands with the device (UDP).
  std::unique_ptr<Socket> socket_command_;
  // The bidirectional communication socket for receiving data with the device (UDP).
  std::unique_ptr<Socket> socket_data_;

  // Cached packet buffer for receiving lidar commands and data. The actual size of the data
  // received is propagated internally. The raw data buffer usage is necessary to comply with the
  // socket API.
  std::vector<char> raw_packet_buffer_;

  // The node time of the last point cloud message posting.
  int64_t previous_message_sent_acq_time_;

  // Monotonically increasing Livox UDP frame sequence number, only applicable to the commands.
  uint16_t sequence_command_;

  // Seeded CRC16 for error detection in the frame preamble. The CRC16 may only be initialized with
  // a seed, hence we must proxy it since we want a delayed initialization in the start function.
  std::optional<FastCRC16> crc16_;
  // Seeded CRC32 for error detection in the overall frame. The CRC32 may only be initialized with
  // a seed, hence we must proxy it since we want a delayed initialization in the start function.
  std::optional<FastCRC32> crc32_;

  // We translate the lidar time to the system with a constant driver time offset from the first
  // data sample. It helps visualizing the clock differences.
  std::optional<uint64_t> driver_acqtime_offset_;

  // The (tick) time (seconds) of the last sent heartbeat.
  double last_sent_hearbeat_time_;
  // Whether we expect an acknowledgment for the last heartbeat.
  bool hearbeat_response_pending_;


  // State to track if data needs to be allocated. Reallocation occurs after publishing
  // once enough data has been collected.
  bool allocate_data_;
  // Sample cloud to accumulate positions
  SampleCloud3f positions_;
  // Sample Cloud to accumulate intensities
  SampleCloud1f intensities_;

  std::optional<PointCloudProto::Builder> builder_;

  // We keep track of how many batches we have accumulated so far and will send the point cloud
  // once we reached the configured batch amount. When changed at runtime, the expected batch count
  // is updated on the message sending boundary to prevent data race condition on the proto message
  // size.
  size_t current_batch_position_;
  size_t expected_batch_count_;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::LivoxLidar);
