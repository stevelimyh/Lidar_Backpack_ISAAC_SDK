'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import numpy as np
import argparse
import json

from engine.pyalice import Application, Cask
from engine.pyalice.Composite import create_composite_message
'''
Moves a robot arm based on joint waypoints to pickup and dropoff a box between two pre-defined
locations repeatedly. In the UR10 use case it also visualizes 3d pose estimation of KLTSmall
boxes in Sight, though the perception result is not used in motion control. This is tested with
omniverse kit isaac sim.
'''


def create_composite_waypoint(name, quantities, values):
    '''Creates a CompositeProto message with name as uuid'''
    msg = create_composite_message(quantities, values)
    msg.uuid = name
    return msg


def create_composite_atlas_ur10(cask_root, joints):
    '''Creates composite atlas cask with waypoints for ur10. Tested with ovkit sim'''
    if len(joints) != 6:
        raise ValueError("UR10 should have 6 joints, got {}".format(len(joints)))

    cask = Cask(cask_root, writable=True)
    # at joint waypoints
    quantities = [[x, "position", 1] for x in joints]

    CART_OBSERVE_WAYPOINT = np.array([3.460, -1.552, 1.810, 4.902, -1.494, 0.796],
                                     dtype=np.dtype("float64"))
    CART_ALIGN_WAYPOINT = np.array([3.4169, -0.5545,  1.7097,  3.5572, -1.5708,  1.8461],
                                   dtype=np.dtype("float64"))
    CART_DROPOFF_WAYPOINT = np.array([3.4169, -0.4016,  1.6376,  3.4764, -1.5708,  1.8461],
                                     dtype=np.dtype("float64"))

    DOLLY_OBSERVE_WAYPOINT = np.array([5.9866, -1.062, 1.251, 5.164, -1.716, 0.217],
                                      dtype=np.dtype("float64"))
    DOLLY_ALIGN_WAYPOINT = np.array([5.9866, -0.3981,  1.3259,  3.7847, -1.5708,  1.2853],
                                    dtype=np.dtype("float64"))
    DOLLY_DROPOFF_WAYPOINT = np.array([5.9866, -0.259 ,  1.2457,  3.7257, -1.5708,  1.2856],
                                      dtype=np.dtype("float64"))

    cask.write_message(create_composite_waypoint("cart_observe", quantities, CART_OBSERVE_WAYPOINT))
    cask.write_message(create_composite_waypoint("cart_align", quantities, CART_ALIGN_WAYPOINT))
    cask.write_message(create_composite_waypoint("cart_dropoff", quantities, CART_DROPOFF_WAYPOINT))
    cask.write_message(
        create_composite_waypoint("dolly_observe", quantities, DOLLY_OBSERVE_WAYPOINT))
    cask.write_message(create_composite_waypoint("dolly_align", quantities, DOLLY_ALIGN_WAYPOINT))
    cask.write_message(
        create_composite_waypoint("dolly_dropoff", quantities, DOLLY_DROPOFF_WAYPOINT))

    quantities = [[x, "none", 1] for x in ["pump", "valve", "gripper"]]
    SUCTION_ON_WAYPOINT = np.array([1.0, 0.0, 1.0], dtype=np.dtype("float64"))
    SUCTION_OFF_WAYPOINT = np.array([0.0, 1.0, 0.0], dtype=np.dtype("float64"))
    VALVE_OFF_WAYPOINT = np.array([0.0, 0.0, 0.0], dtype=np.dtype("float64"))
    cask.write_message(create_composite_waypoint("suction_on", quantities, SUCTION_ON_WAYPOINT))
    cask.write_message(create_composite_waypoint("suction_off", quantities, SUCTION_OFF_WAYPOINT))
    cask.write_message(create_composite_waypoint("valve_off", quantities, VALVE_OFF_WAYPOINT))


def create_composite_atlas(cask_root, arm, joints):
    '''Creates composite atlas cask based on the arm'''
    if arm == "ur10":
        create_composite_atlas_ur10(cask_root, joints)
    else:
        raise ValueError("Waypoints for {} not available".format(arm))


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description="Sortbot Demo")
    parser.add_argument("--arm", help="Type of arm used.", choices=["ur10"], default="ur10")
    parser.add_argument("--cask", help="Path to output atlas", default="/tmp/shuffle_box_waypoints")
    args = parser.parse_args()

    # get kinematic file and joints
    kinematic_file = "apps/assets/kinematic_trees/{}.kinematic.json".format(args.arm)
    joints = []
    with open(kinematic_file, 'r') as fd:
        kt = json.load(fd)
        for link in kt['links']:
            if 'motor' in link and link['motor']['type'] != 'constant':
                joints.append(link['name'])

    # create composite atlas
    create_composite_atlas(args.cask, args.arm, joints)

    # Create and start the app
    app = Application(name="Shuffle Box")
    # load bebavior subgraph. this contains the sequency behavior to move the arm between
    # waypoints and control suction on/off
    app.load("apps/samples/manipulation/shuffle_box_behavior.subgraph.json", prefix="behavior")
    behavior_interface = app.nodes["behavior.interface"]["subgraph"]
    app.nodes["behavior.atlas"]["CompositeAtlas"].config.cask = args.cask
    app.load("packages/planner/apps/multi_joint_lqr_control.subgraph.json", prefix="lqr")

    # load multi joint lqr control subgraph
    lqr_interface = app.nodes["lqr.subgraph"]["interface"]
    kinematic_tree = app.nodes["lqr.kinematic_tree"]["KinematicTree"]
    lqr_planner = app.nodes["lqr.local_plan"]["MultiJointLqrPlanner"]
    app.connect(behavior_interface, "joint_target", lqr_interface, "joint_target")
    kinematic_tree.config.kinematic_file = kinematic_file
    lqr_planner.config.speed_min = [-1.5] + [-1.0] * (len(joints) - 1)
    lqr_planner.config.speed_max = [1.5] + [1.0] * (len(joints) - 1)
    lqr_planner.config.acceleration_min = [-1.5] + [-1.0] * (len(joints) - 1)
    lqr_planner.config.acceleration_max = [1.5] + [1.0] * (len(joints) - 1)

    # load perception subgraph for KLTBox detection in ur10 usecase
    use_perception = False
    if args.arm == "ur10":
        use_perception = True
        app.load(
            "packages/object_pose_estimation/apps/pose_cnn_decoder" \
            "/detection_pose_estimation_cnn_inference.subgraph.json",
            prefix="detection_pose_estimation")
        perception_interface = app.nodes["detection_pose_estimation.interface"]["Subgraph"]
        app.load("apps/samples/manipulation/shuffle_box_detection_pose_estimation.config.json")

    # load sim tsubgraph for tcp connection.
    app.load("packages/navsim/apps/navsim_tcp.subgraph.json", "simulation")
    sim_in = app.nodes["simulation.interface"]["input"]
    sim_out = app.nodes["simulation.interface"]["output"]
    app.connect(sim_out, "joint_state", lqr_interface, "joint_state")
    app.connect(sim_out, "joint_state", behavior_interface, "joint_state")
    app.connect(sim_out, "io_state", behavior_interface, "io_state")
    app.connect(lqr_interface, "joint_command", sim_in, "joint_position")
    app.connect(behavior_interface, "io_command", sim_in, "io_command")
    if use_perception:
        app.connect(sim_out, "color", perception_interface, "color")
    viewers = app.add("viewers")
    depth_viewer = viewers.add(app.registry.isaac.viewers.DepthCameraViewer, "DepthViewer")
    app.connect(sim_out, "depth", depth_viewer, "depth_listener")
    depth_viewer.config.max_visualization_depth = 3

    # run app
    app.run()
