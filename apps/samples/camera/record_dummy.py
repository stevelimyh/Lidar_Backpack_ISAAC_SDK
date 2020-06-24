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
import time


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Logs data produced by a dummy camera.')
    parser.add_argument('--base_directory', dest='base_directory', default='/tmp',
                        help='The directory in which log files will be stored')
    parser.add_argument('--fps', dest='fps', type=float, default=3.0,
                        help='The framerate at which to record')
    parser.add_argument('--rows', dest='rows', type=int, default=720,
                        help='The vertical resolution of the camera images')
    parser.add_argument('--cols', dest='cols', type=int, default=1280,
                        help='The horizontal resolution of the camera images')
    args, _ = parser.parse_known_args()

    app = Application(name="record_dummy", modules=["message_generators"])

    # Create recorder node
    recorder = app.add("recorder").add(app.registry.isaac.alice.Recorder)
    recorder.config.base_directory = args.base_directory

    # Create dummy camera codelet
    camera = app.add("cam").add(app.registry.isaac.message_generators.CameraGenerator)
    camera.config.rows = args.rows
    camera.config.cols = args.cols
    camera.config.tick_period = str(args.fps) + "Hz"
    app.connect(camera, "color_left", recorder, "color")
    app.connect(camera, "depth", recorder, "depth")

    app.run()
