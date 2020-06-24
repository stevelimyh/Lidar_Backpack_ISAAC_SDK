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


def populate_parser(parser):
    ''' Sets up parser for core arguments '''
    parser.add_argument(
        '--config',
        dest='config',
        default=
            'packages/detect_net/apps/detect_net_industrial_dolly.config.json',
        help='Config file to load. Will be overwritten if other model-specific parameters are '\
            'provided')
    parser.add_argument(
        '--cask_directory',
        dest='cask_directory',
        default='external/industrial_dolly_pose_estimation_data/dolly_logs_2020_04_16',
        help='The cask directory used to replay data from')
    parser.add_argument('--detection_model',
                        dest='detection_model_file_path',
                        help='Path to detection model (.etlt)')
    parser.add_argument('--etlt_password',
                        dest='etlt_password',
                        help='Password for detection .etlt model')
    parser.add_argument('--confidence_threshold',
                        dest='confidence_threshold',
                        type=float,
                        help='Confidence threshold for detection model')
    parser.add_argument('--nms_threshold',
                        dest='nms_threshold',
                        type=float,
                        help='Non-maximum suppression threshold for detection model')
    parser.add_argument('--rows',
                        dest='rows',
                        type=int,
                        default=720,
                        help='Number of rows (height) of images')
    parser.add_argument('--cols',
                        dest='cols',
                        type=int,
                        default=1280,
                        help='Number of cols (width) of images')
    parser.add_argument(
        '--mode',
        dest='mode',
        type=str,
        default='cask',
        help='Running mode. Valid values: cask, realsense, v4l, sim, image',
    )
    return parser


def populate_sim_parser(parser):
    ''' Sets up parser for sim only arguments '''
    parser.add_argument('--image_channel',
                        dest='image_channel',
                        default='color',
                        help='Name of color image channel coming from simulation')
    return parser


def populate_camera_parser(parser):
    ''' Sets up parser for camera only arguments '''
    parser.add_argument('--fps',
                        dest='fps',
                        type=int,
                        default=15,
                        help='The framerate to set for rgb and depth image channels')
    return parser


def populate_image_parser(parser):
    ''' Sets up parser for image only arguments '''
    parser.add_argument(
        '--image_directory',
        dest='image_directory',
        default=
        'external/industrial_dolly_pose_estimation_data/dolly_sample_images/dolly_color_1.png',
        help='GLOB pattern for files to use for inference')
    parser.add_argument(
        '--focal_length',
        dest='focal_length',
        type=float,
        default=925.74,
        help=
            'Focal length of camera used to capture images. Assumes this value is the same'\
            'for both height and width camera dimensions')
    parser.add_argument('--optical_center_rows',
                        dest='optical_center_rows',
                        type=int,
                        default=360,
                        help='Height of optical center')
    parser.add_argument('--optical_center_cols',
                        dest='optical_center_cols',
                        type=int,
                        default=640,
                        help='Width of optical center')
    return parser


def main(args):
    app = Application(name="detect_net_inference")

    # Load subgraph and get interface node
    app.load("packages/detect_net/apps/detect_net_inference.subgraph.json",
        prefix="detect_net_inference")
    detect_net_inferface = app.nodes["detect_net_inference.subgraph"]\
        .components["interface"]

    # Load configuration
    app.load(args.config)

    # Configure detection model
    detection_model = app.nodes["detect_net_inference.tensor_r_t_inference"]\
        .components["isaac.ml.TensorRTInference"]
    if args.detection_model_file_path is not None:
        detection_model.config.model_file_path = args.detection_model_file_path
    if args.etlt_password is not None:
        detection_model.config.etlt_password = args.etlt_password

    # Configure detection decoder
    decoder = app.nodes["detect_net_inference.detection_decoder"]\
        .components["isaac.detect_net.DetectNetDecoder"]
    decoder.config.output_scale = [args.rows, args.cols]
    if args.confidence_threshold is not None:
        decoder.config.confidence_threshold = args.confidence_threshold
    if args.nms_threshold is not None:
        decoder.config.non_maximum_suppression_threshold = args.nms_threshold

    if args.mode == 'cask':
        # Load replay subgraph and configure interface node
        app.load("packages/record_replay/apps/replay.subgraph.json", prefix="replay")
        replay_interface = app.nodes["replay.interface"].components["output"]
        replay_interface.config.cask_directory = args.cask_directory

        # Connect the output of the replay subgraph to the detection subgraph
        app.connect(replay_interface, "color", detect_net_inferface, "image")
    elif args.mode == 'sim':
        # Load simulation subgraph and get interface node
        app.load("packages/navsim/apps/navsim_training.subgraph.json",\
             prefix="simulation")
        simulation_interface = app.nodes["simulation.interface"].components["output"]

        # Connect the output of the simulation with user-specified channel to the detection subgraph
        app.connect(simulation_interface, args.image_channel, detect_net_inferface, "image")
    elif args.mode == 'realsense':
        app.load_module('realsense')
        # Create and configure realsense camera codelet
        camera = app.add("camera").add(app.registry.isaac.RealsenseCamera)
        camera.config.rows = args.rows
        camera.config.cols = args.cols
        camera.config.color_framerate = args.fps
        camera.config.depth_framerate = args.fps
        camera.config.enable_ir_stereo = False

        # Connect the output of the camera node to the detection subgraph
        app.connect(camera, "color", detect_net_inferface, "image")
    elif args.mode == 'v4l':
        app.load_module('sensors:v4l2_camera')
        # Create and configure V4L camera codelet
        camera = app.add("camera").add(app.registry.isaac.V4L2Camera)
        camera.config.device_id = 0
        camera.config.rows = args.rows
        camera.config.cols = args.cols
        camera.config.rate_hz = args.fps

        # Connect the output of the camera node to the detection subgraph
        app.connect(camera, "frame", detect_net_inferface, "image")
    elif args.mode == 'image':
        app.load_module('message_generators')
        # Create feeder node
        feeder = app.add("feeder").add(app.registry.isaac.message_generators.ImageLoader)
        feeder.config.color_glob_pattern = args.image_directory
        feeder.config.tick_period = "1Hz"
        feeder.config.focal_length = [args.focal_length, args.focal_length]
        feeder.config.optical_center = [args.optical_center_rows, args.optical_center_cols]
        feeder.config.distortion_coefficients = [0.01, 0.01, 0.01, 0.01, 0.01]

        # Connect the output of the image feeder node to the detection subgraph
        app.connect(feeder, "color", detect_net_inferface, "image")
    else:
        raise ValueError('Not supported mode {}'.format(args.mode))
    app.run()


if __name__ == '__main__':
    parser = populate_parser(
        argparse.ArgumentParser(description='Run DetectNetv2 inference'))
    parser = populate_sim_parser(parser)
    parser = populate_camera_parser(parser)
    parser = populate_image_parser(parser)
    args, _ = parser.parse_known_args()
    main(args)
