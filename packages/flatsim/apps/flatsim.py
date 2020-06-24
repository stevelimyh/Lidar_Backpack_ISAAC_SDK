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
import random
import sys


DEMOS = [
    "demo_1",
    "demo_2",
    "demo_3",
    "demo_4"
]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='flatsim is a flat-world simulator for navigation')
    parser.add_argument('--demo', dest='demo',
                        help='The scenario which will be used for flatsim')
    args, _ = parser.parse_known_args()

    demo = args.demo if args.demo is not None else random.choice(DEMOS)

    app = Application(name="flatsim")
    app.load("packages/flatsim/apps/flatsim.subgraph.json", prefix="flatsim")
    app.load("packages/flatsim/apps/{}.json".format(demo))

    app.run()
