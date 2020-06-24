'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import numpy as np
from engine.pyalice import Cask
from engine.pyalice.Composite import create_composite_message
'''
A helper app for the composite_atlas.py test application. It uses the Python API to write test
composite waypoints to a cask using CompositeProto.
'''


def create_composite_waypoint(name, quantities, values):
    '''Creates a CompositeProto message which contains given quantities as waypoint'''
    msg = create_composite_message(quantities, values)
    msg.uuid = name
    return msg


if __name__ == '__main__':
    cask = Cask("waypoints", writable=True)
    joints = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
        'wrist_2_joint', 'wrist_3_joint'
    ]
    quantities = [[x, "position", 1] for x in joints]

    wp_cart = np.array([3.233, -1.486, 1.798, 4.850, -1.522, 0.558], dtype=np.dtype("float64"))
    wp_cart_dropoff = np.array(
        [3.394, -0.632, 1.660, 3.656, -1.521, 1.877], dtype=np.dtype("float64"))
    wp_dolly = np.array([6.053, -1.056, 1.244, 5.290, -1.722, 0.389], dtype=np.dtype("float64"))
    wp_dolly_dropoff = np.array(
        [5.889, -0.386, 1.451, 3.706, -1.524, 1.153], dtype=np.dtype("float64"))

    cask.write_message(create_composite_waypoint("cart", quantities, wp_cart))
    cask.write_message(create_composite_waypoint("cart_dropoff", quantities, wp_cart_dropoff))
    cask.write_message(create_composite_waypoint("dolly", quantities, wp_dolly))
    cask.write_message(create_composite_waypoint("dolly_dropoff", quantities, wp_dolly_dropoff))
