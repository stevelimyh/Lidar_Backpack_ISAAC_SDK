'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from engine.pyalice import Application
import argparse

if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description="GTC 2020 demo")
    parser.add_argument(
        "--demo_subgraph_name",
        help="The name of the demo subgraph node in the main app.",
        default="delivery")
    parser.add_argument(
        "--map_json",
        help="Path to the json file for the map.",
        default="apps/assets/maps/virtual_factory_1.json")
    parser.add_argument(
        "--pose2_planner",
        help="Uses the Pose2GraphPlanner instead of the default global planner if enabled",
        default=True)
    parser.add_argument(
        "--mission_robot_name",
        help="Accept missions from the remote mission server for the robot with the given name")
    parser.add_argument(
        "--mission_host",
        help="The ip address or hostname of the host to connect to and receive missions from",
        default="localhost")
    parser.add_argument(
        "--mission_port",
        help="The TCP port to connect to the mission server",
        type=int,
        default=9998)
    args = parser.parse_args()

    # Create and start the app
    more_jsons = args.map_json
    more_jsons += ",packages/navsim/robots/str4.json"
    app = Application(
        app_filename="packages/cart_delivery/apps/cart_delivery.app.json", more_jsons=more_jsons)
    app.load("packages/cart_delivery/apps/navigation.config.json", prefix=args.demo_subgraph_name)
    app.load(
        "packages/cart_delivery/apps/detection_pose_estimation.config.json",
        prefix=args.demo_subgraph_name)
    if args.pose2_planner:
        app.load(
            "packages/cart_delivery/apps/pose2_planner.config.json", prefix=args.demo_subgraph_name)
    if args.mission_robot_name:
        # Load the mission subgraph and set the config based on the input parameters
        app.load("packages/behavior_tree/apps/missions.graph.json")
        app.nodes["tcp_client"].components["JsonTcpClient"].config["host"] = args.mission_host
        app.nodes["tcp_client"].components["JsonTcpClient"].config["port"] = args.mission_port
        app.nodes["mission_control"].components["NodeGroup"].config["node_names"] = \
            [args.demo_subgraph_name + ".sequence_behavior"]
        app.nodes["robot_name"].components["JsonMockup"].config["json_mock"] = \
            {"text":args.mission_robot_name}
        run_on_start = app.nodes[args.demo_subgraph_name + ".run_on_start"]
        # Change the start behavior to the mission behavior
        nodes = run_on_start.components["NodeGroup"].config["node_names"]
        run_on_start.components["NodeGroup"].config["node_names"] = nodes + ["mission_control"]
        run_on_start.components["SwitchBehavior"].config["desired_behavior"] = "mission_control"
    app.start_wait_stop()
