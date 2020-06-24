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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="""
Description: Apriltag detection using different cameras - python sample application
Example usage(for Jetson):
             $python3 apps/samples/april_tags/april_tags_python.py
             $python3 apps/samples/april_tags/april_tags_python.py --camera realsense --resolution 1280x720 --framerate 30
             $python3 apps/samples/april_tags/april_tags_python.py --camera v4l2 --resolution 1920x1080 --framerate 15 --device_id 3""",
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '--camera',
        dest='camera',
        action='store',
        default='zed',
        choices=['zed', 'realsense', 'v4l2'],
        help='Camera type')
    parser.add_argument(
        '--resolution',
        dest='resolution',
        action='store',
        default='1280x720',
        help='Camera resolution')
    parser.add_argument(
        '--framerate',
        dest='framerate',
        action='store',
        type=int,
        default=60,
        help='Camera framerate')
    parser.add_argument(
        '--device_id', dest='device_id', action='store', type=int, help='Camera device id')
    args = parser.parse_args()
    # Create april_tag_python application
    app = Application(
        name="april_tags_python",
        modules=[
            "//packages/perception:april_tags", "realsense", "sensors:v4l2_camera", "viewers", "zed"
        ])
    # Setup camera node
    camera = None
    if args.camera == "zed":
        camera = app.add('input_images').add(app.registry.isaac.ZedCamera)
        camera.config.resolution = args.resolution
        camera.config.tick_period = '{tick_period}Hz'.format(tick_period=args.framerate)
        camera_out_channel = "left_camera_rgb"
    elif args.camera == "realsense":
        camera = app.add('input_images').add(app.registry.isaac.RealsenseCamera)
        camera.config.cols, camera.config.rows = tuple(
            [int(arg) for arg in args.resolution.split('x')])
        camera.config.color_framerate = args.framerate
        camera_out_channel = "color"
    elif args.camera == "v4l2":
        camera = app.add('input_images').add(app.registry.isaac.V4L2Camera)
        if args.device_id == None:
            raise ValueError('Could not set None. Please provide device id')
        camera.config.device_id = args.device_id
        camera.config.cols, camera.config.rows = tuple(
            [int(arg) for arg in args.resolution.split('x')])
        camera.config.rate_hz = args.framerate
        camera_out_channel = "frame"
    else:
        raise ValueError('Not supported Camera type {}'.format(args.camera))
    # Setup april tag node
    tags_detection = app.add('april_tags_detection').add(
        app.registry.isaac.perception.AprilTagsDetection)
    tags_detection.config.max_tags = 50
    fiducials_viewer = app.nodes['april_tags_detection'].add(
        app.registry.isaac.viewers.FiducialsViewer)
    # Setup viewer node
    viewer = app.add('image_viewers').add(app.registry.isaac.viewers.ColorCameraViewer)
    # Connect message channels
    app.connect(camera, camera_out_channel, tags_detection, "image")
    app.connect(tags_detection, "april_tags", fiducials_viewer, "fiducials")
    app.connect(camera, camera_out_channel, viewer, "color_listener")

    app.load("apps/samples/april_tags/april_tags_python.config.json")

    app.run()
