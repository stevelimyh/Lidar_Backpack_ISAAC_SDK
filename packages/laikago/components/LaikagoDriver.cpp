/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "LaikagoDriver.hpp"

#include <memory>

#include "engine/alice/components/Failsafe.hpp"
#include "engine/gems/state/io.hpp"
#include "messages/state/holonomic_base.hpp"

namespace isaac {
namespace laikago {
namespace {
// Input linear speed bound
constexpr double kMaxSafeLinearSpeedLimit = 1.0;
// Input angular speed bound
constexpr double kMaxSafeAngularSpeedLimit = 1.0;
// Command standing
constexpr int kStanding = 1;
// Command walking
constexpr int kWalking = 2;
// Convert mlisecond to nanosecond
constexpr int64_t kMiliSecondsToNanoSeconds = 1000000;
}  // namespace

void LaikagoDriver::start() {
    failsafe_ = node()->getComponent<alice::Failsafe>();
    control_ = std::make_unique<::laikago::Control>(::laikago::HIGHLEVEL);
    udp_ = std::make_unique<::laikago::UDP>(::laikago::HIGH_CMD_LENGTH,
                                            ::laikago::HIGH_STATE_LENGTH);
    cmd_.levelFlag = 0;
    state_.levelFlag = 0;
    control_->InitCmdData(cmd_);
    tickPeriodically();
}

void LaikagoDriver::tick() {
    udp_->Recv();
    udp_->GetState(state_);

    cmd_.forwardSpeed = 0.0f;
    cmd_.sideSpeed = 0.0f;
    cmd_.rotateSpeed = 0.0f;

    cmd_.mode = kStanding;
    cmd_.roll  = 0.0f;
    cmd_.pitch = 0.0f;
    cmd_.yaw = 0.0f;

    if (rx_base_command().available()) {
        // Get the commanded values and make sure they are safe
        messages::HolonomicBaseControls command;
        if (!FromProto(rx_base_command().getProto(), rx_base_command().buffers(), command)) {
            reportFailure("Received invalid command message");
            return;
        }
        Vector2d safe_linear_speed{command.speed_x(), command.speed_y()};
        double safe_angular_speed = command.angular_speed();
        const double speed_limit_linear = get_speed_limit_linear();
        const double speed_limit_angular = get_speed_limit_angular();
        if (speed_limit_linear <= 0.0 || speed_limit_linear >= kMaxSafeLinearSpeedLimit) {
            reportFailure(
                    "Linear speed limit is %f. It needs to be a positive number smaller than %f.",
                    speed_limit_linear, kMaxSafeLinearSpeedLimit);
            return;
        }
        if (speed_limit_angular <= 0.0 || speed_limit_angular >= kMaxSafeAngularSpeedLimit) {
            reportFailure(
                    "Angular speed limit is %f. It needs to be a positive number smaller than %f.",
                    speed_limit_angular, kMaxSafeAngularSpeedLimit);
            return;
        }
        if (safe_linear_speed.norm() > speed_limit_linear) {
            const double scale = speed_limit_linear / safe_linear_speed.norm();
            safe_linear_speed *= scale;
        }
        if (std::abs(safe_angular_speed) > speed_limit_angular) {
            safe_angular_speed = std::copysign(speed_limit_angular, command.angular_speed());
        }

        // stop robot if the failsafe is triggered
        if (!failsafe_->isAlive()) {
            safe_linear_speed = Vector2d::Zero();
            safe_angular_speed = 0.0;
        }
        // Send commands to the robot
        show("command.linear_speed_x", safe_linear_speed.x());
        show("command.linear_speed_y", safe_linear_speed.y());
        show("command.angular_speed", safe_angular_speed);
        show("state.linear_speed_x", state_.forwardSpeed);
        show("state.linear_speed_y", state_.sideSpeed);
        show("state.angular_speed", DegToRad(state_.rotateSpeed));

        const double min_command_speed = get_min_command_speed();
        if (std::abs(safe_linear_speed.norm()) > min_command_speed ||
            std::abs(safe_angular_speed) > min_command_speed) {
            cmd_.mode = kWalking;
            if (safe_linear_speed.x() >= 0) {
                cmd_.forwardSpeed = safe_linear_speed.x();
            } else {
                cmd_.forwardSpeed = get_scale_back_speed() * safe_linear_speed.x();
            }
            cmd_.sideSpeed = get_scale_side_speed() * safe_linear_speed.y();
            cmd_.rotateSpeed = safe_angular_speed;
        }
    }

    udp_->Send(cmd_);

    // Publishes current state of base
    messages::HolonomicBaseDynamics state;
    state.speed_x() = state_.forwardSpeed;
    state.speed_y() = state_.sideSpeed;
    state.angular_speed() = DegToRad(state_.rotateSpeed);
    state.acceleration_x() = 0.0;
    state.acceleration_y() = 0.0;
    ToProto(state, tx_base_state().initProto(), tx_base_state().buffers());
    tx_base_state().publish();

    // Pass laikago imu to imu proto
    auto imu_builder = tx_imu().initProto();

    // set accelerometer data
    imu_builder.setLinearAccelerationX(state_.imu.acceleration[0]);
    imu_builder.setLinearAccelerationY(state_.imu.acceleration[1]);
    imu_builder.setLinearAccelerationZ(state_.imu.acceleration[2]);

    // set gyro data
    imu_builder.setAngularVelocityX(state_.imu.gyroscope[0]);
    imu_builder.setAngularVelocityY(state_.imu.gyroscope[1]);
    imu_builder.setAngularVelocityZ(state_.imu.gyroscope[2]);

    tx_imu().publish();
}

void LaikagoDriver::stop() {
  // Set to stance pose
  cmd_.forwardSpeed = 0.0f;
  cmd_.sideSpeed = 0.0f;
  cmd_.rotateSpeed = 0.0f;

  cmd_.mode = kStanding;
  cmd_.roll  = 0.0f;
  cmd_.pitch = 0.0f;
  cmd_.yaw = 0.0f;

  udp_->Send(cmd_);
  control_ = nullptr;
  udp_ = nullptr;
}

}  // namespace laikago
}  // namespace isaac
