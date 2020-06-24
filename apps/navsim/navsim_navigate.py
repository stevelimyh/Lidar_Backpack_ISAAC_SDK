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
    parser = argparse.ArgumentParser(description="Navsim navigation app")
    parser.add_argument(
        "--map_json",
        help="The path to the map json to load",
        default="apps/assets/maps/virtual_small_warehouse.json")
    parser.add_argument(
        "--robot_json",
        help="The path to the robot json to load",
        default="packages/navsim/robots/carter.json")
    parser.add_argument(
        "--more",
        help="A comma separated list of additional json files to load")
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
    more_jsons = args.map_json + "," + args.robot_json
    if args.more:
        more_jsons += "," + args.more
    app_path = "apps/navsim/navsim_navigate.app.json"
    app = Application(app_filename=app_path, more_jsons=more_jsons)

    if args.mission_robot_name:
        # Load the mission subgraph and set the config based on the input parameters
        app.load(
            "packages/behavior_tree/apps/missions.graph.json")
        app.nodes["tcp_client"].components["JsonTcpClient"].config["host"] = args.mission_host
        app.nodes["tcp_client"].components["JsonTcpClient"].config["port"] = args.mission_port
        app.nodes["mission_control"].components["NodeGroup"].config["node_names"] = \
            ["goals.goal_behavior"]
        app.nodes["robot_name"].components["JsonMockup"].config["json_mock"] = \
            {"text":args.mission_robot_name}
        run_on_start = app.nodes["goals.run_on_start"]
        # Change the start behavior to the mission behavior
        nodes = run_on_start.components["NodeGroup"].config["node_names"]
        run_on_start.components["NodeGroup"].config["node_names"] = nodes + ["mission_control"]
        run_on_start.components["SwitchBehavior"].config["desired_behavior"] = "mission_control"
        # Send the navigation output back through the json tcp client
        app.connect(app.nodes["navigation.subgraph"].components["interface"], "feedback",
            app.nodes["tcp_client"].components["JsonTcpClient"], "feedback")

    app.start_wait_stop()
