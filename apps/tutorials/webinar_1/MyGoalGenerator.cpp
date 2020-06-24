/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "MyGoalGenerator.hpp"
#include "messages/math.hpp"


namespace isaac {
namespace tutorials {

void MyGoalGenerator::start() {
  tickPeriodically();
  LOG_INFO("Hello Bruh!!");
}

void MyGoalGenerator::tick() {
	//LOG_WARNING("tick!");

	auto goal_proto = tx_my_goal().initProto();
	//
	goal_proto.setStopRobot(false);
	goal_proto.setTolerance(0.1);
	goal_proto.setGoalFrame("world");
	ToProto(Pose2d::Translation(get_desired_location()),goal_proto.initGoal());
	tx_my_goal().publish();

	rx_feedback_goal().processLatestNewMessage(
		[this](auto feedback_proto, int64_t, int64_t){
			const bool arrived = feedback_proto.getHasArrived();
			show("arrived", arrived ? 1.0:0.0);
		});

}

}  // namespace tutorials
}  // namespace isaac
