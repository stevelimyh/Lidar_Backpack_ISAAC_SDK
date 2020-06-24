/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "NavigationRosBridge.hpp"

#include <memory>

#include "packages/ros_bridge/gems/include_before_ros.hpp"

#include "engine/core/assert.hpp"
#include "geometry_msgs/Pose2D.h"
#include "messages/math.hpp"
#include "ros/callback_queue.h"
#include "ros/ros.h"

namespace isaac {
namespace rosbridge {

namespace {
// Callback functor to avoid having a callback function in the bridge class
class CallbackFunctor {
 public:
  explicit CallbackFunctor(NavigationRosBridge* bridge) {
    bridge_ = bridge;
  }
  CallbackFunctor(const CallbackFunctor&) = default;
  ~CallbackFunctor() = default;

  void operator()(const geometry_msgs::Pose2D::ConstPtr& msg) {
    const Pose2d pose = Pose2d::FromXYA(msg->x, msg->y, msg->theta);
    auto goal_proto = bridge_->tx_goal().initProto();
    ToProto(pose, goal_proto.initGoal());
    goal_proto.setGoalFrame("world");
    bridge_->tx_goal().publish();
  }
 private:
  NavigationRosBridge* bridge_;
};
}  // namespace

// Internal struct for holding the ROS node handle and the publisher and subscriber channels
// Note the callback queue. To avoid spinning other codelets it is necessary to generate
// a separate callback queue per codelet.
struct NavigationRosBridge::RosNavigationData {
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::CallbackQueue callbackQueue;
};

void NavigationRosBridge::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // roscore needs to be started externally.
  if (!ros::master::check()) {
    reportFailure("ROS is not running. Please start roscore first.");
    return;
  }

  nav_data_ = std::make_unique<RosNavigationData>();
  nav_data_->node.setCallbackQueue(&(nav_data_->callbackQueue));
  nav_data_->pub = nav_data_->node.advertise<geometry_msgs::Pose2D>(
      get_publisher_channel_name(), get_publisher_queue_size());
  nav_data_->sub = nav_data_->node.subscribe<geometry_msgs::Pose2D>(
      get_subscriber_channel_name(), get_subscriber_queue_size(), CallbackFunctor(this));

  // Because ROS requires that the message queues be manually pumped we need to tick.
  tickPeriodically();
}

void NavigationRosBridge::tick() {
  if (ros::ok()) {
    bool ok;
    const Pose2d world_T_robot = get_world_T_robot(getTickTime(), ok);
    if (ok) {
      geometry_msgs::Pose2D msg;
      msg.x = world_T_robot.translation(0);
      msg.y = world_T_robot.translation(1);
      msg.theta = world_T_robot.rotation.angle();
      nav_data_->pub.publish(msg);
    }
    // Pump the queue for this codelet.
    nav_data_->callbackQueue.callAvailable();
  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void NavigationRosBridge::stop() {
  if (nav_data_) {
    nav_data_->pub.shutdown();
    nav_data_->sub.shutdown();
    nav_data_ = nullptr;
  }
}

NavigationRosBridge::~NavigationRosBridge() {
}

NavigationRosBridge::NavigationRosBridge() {
}

}  // namespace rosbridge
}  // namespace isaac
