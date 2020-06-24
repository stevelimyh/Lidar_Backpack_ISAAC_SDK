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
    parser = argparse.ArgumentParser(description='Logs data from a RealSense device.')
    parser.add_argument('--base_directory', dest='base_directory', default='/tmp',
                        help='The directory in which log files will be stored')
    parser.add_argument('--fps', dest='fps', type=int, default=30,
                        help='The framerate at which to record')
    parser.add_argument('--mode', dest='mode', choices=['720p', '640x480'], default='720p',
                        help='The resolution of the camera images')
    args, _ = parser.parse_known_args()

    app = Application(name="record_realsense", modules=["realsense"])

    if args.mode == '720p':
      rows = 720
      cols = 1280
    elif args.mode == '640x480':
      rows = 640
      cols = 480
    else:
      raise ValueError('Not supported camera resolution')

    # Create recorder node
    recorder = app.add("recorder").add(app.registry.isaac.alice.Recorder)
    recorder.config.base_directory = args.base_directory

    # Create realsense camera codelet
    camera = app.add("cam").add(app.registry.isaac.RealsenseCamera)
    camera.config.rows = rows
    camera.config.cols = cols
    camera.config.color_framerate = args.fps
    camera.config.depth_framerate = args.fps
    app.connect(camera, "color", recorder, "color")
    app.connect(camera, "depth", recorder, "depth")

    app.run()
