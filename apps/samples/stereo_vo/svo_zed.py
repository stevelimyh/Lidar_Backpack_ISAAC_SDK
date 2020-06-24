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
    parser = argparse.ArgumentParser(
        description='Demonstrates the Stereo Visual Odometry tracking'
        '  using the live stereo image feed obtained from the ZED (Mini) camera.')
    parser.add_argument('--imu',
                        dest='imu',
                        action='store_true',
                        help='Enables the support for the on-board camera IMU.')
    parser.add_argument('--no-imu',
                        dest='imu',
                        action='store_false',
                        help='Disables the support for the on-board camera IMU.')
    parser.set_defaults(imu=False)
    args = parser.parse_args()

    app = Application(name="svo_zed",
                      modules=["perception:stereo_visual_odometry", "utils", "viewers", "zed"])

    camera = app.add('camera').add(app.registry.isaac.ZedCamera)
    camera.config.enable_factory_rectification = True
    camera.config.enable_imu = args.imu
    camera.config.resolution = "672x376"
    camera.config.gray_scale = True
    camera.config.rgb = False
    camera.config.tick_period = "60Hz"

    splitter_left = app.add('camera_splitter_left').add(
        app.registry.isaac.utils.ColorCameraProtoSplitter)
    splitter_left.config.only_pinhole = False

    splitter_right = app.add('camera_splitter_right').add(
        app.registry.isaac.utils.ColorCameraProtoSplitter)
    splitter_right.config.only_pinhole = False

    tracker = app.add('tracker').add(app.registry.isaac.StereoVisualOdometry)
    tracker.config.horizontal_stereo_camera = True
    tracker.config.process_imu_readings = args.imu
    tracker.config.lhs_camera_frame = "zed_left_camera"
    tracker.config.rhs_camera_frame = "zed_right_camera"
    tracker.config.imu_frame = "zed_imu"

    if (args.imu):
        camera_imu_reader = app.nodes['camera'].add(app.registry.isaac.zed.ZedImuReader)
        camera_imu_reader.config.tick_period = "300Hz"
        app.connect(camera_imu_reader, "imu_raw", tracker, "imu")

    viewer = app.add('viewers').add(app.registry.isaac.viewers.ColorCameraViewer)
    viewer.config.reduce_scale = 2

    app.connect(camera, "left_camera_gray", splitter_left, "color_camera")
    app.connect(splitter_left, "image", tracker, "left_image")
    app.connect(splitter_left, "intrinsics", tracker, "left_intrinsics")

    app.connect(camera, "right_camera_gray", splitter_right, "color_camera")
    app.connect(splitter_right, "image", tracker, "right_image")
    app.connect(splitter_right, "intrinsics", tracker, "right_intrinsics")

    app.connect(camera, "left_camera_gray", viewer, "color_listener")

    app.load("apps/samples/stereo_vo/svo_zed.config.json")

    app.run()
