'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
"""
Isaac tool to freeze tensorflow graph from checkpoint files.

Loads checkpoint weight and graph files and writes a single frozen tensorflow
graph file.

usage: freeze_tensorflowgraph.py [-h] [--out OUT]
                                 --output_node_name OUTPUT_NODE_NAME
                                 INPUT_CHECKPOINT_PATH

positional arguments:
  INPUT_CHECKPOINT_PATH      Path to the input checkpoint files (*.data, *.index, *.meta)

optional arguments:
  -h, --help            show this help message and exit
  --out OUT             Path to the output Tensorflow model (.pb)
  --output_node_name OUTPUT_NODE_NAME
                        Output node name. This parameter must be provided.
"""

import argparse, sys, os.path
import tensorflow as tf
from tensorflow.python.tools import freeze_graph

def main(args):
    """
    Loads checkpoint weight and graph files and writes a single frozen tensorflow
    graph file.
    Output file is saved in the same path and name as input checkpoint file but
    with a .pb extension.

    Arguments:
    args: the parsed command line arguments
    """

    ckpt_file = args.input_checkpoint_path
    ckpt_dir = ckpt_file.replace(ckpt_file.split("/")[-1], '')

    if not args.output_node_name:
        sys.exit('Failed to provide output nodes of the graph. Exiting.')

    # Saving model.pb file from the checkpoint files
    if args.out:
        frozen_file = os.path.join("{}-frozen.pb".format(args.out[0]))
    else:
        # Save the model in the same folder as checkpoint
        frozen_file = os.path.join("{}-frozen.pb".format(ckpt_file))

    freeze_graph.freeze_graph(
        input_graph=os.path.join(ckpt_dir, 'graph.pb'),
        input_saver="",
        input_binary=True,
        input_checkpoint=ckpt_file,
        output_node_names=args.output_node_name[0],
        restore_op_name="save/restore_all",
        filename_tensor_name="save/Const:0",
        output_graph=frozen_file,
        clear_devices=True,
        initializer_nodes="")
    print("Saved frozen model at {}.".format(frozen_file))

def parse_arguments():
    """
    Set required input arguments and add functionality to parse them

    Returns:
    The arguments parsed from the command line
    """
    parser = argparse.ArgumentParser(
        description="Isaac tool to freeze tensorflow graph from checkpoint files.\n"
        "Loads the checkpoint files for a step."
        "Writes a single tensorflow frozen(.pb) model.)")
    parser.add_argument(
        "input_checkpoint_path",
        metavar="INPUT_CHECKPOINT_PATH",
        help="Path to the input checkpoint files")
    parser.add_argument(
        "--out",
        action='append',
        metavar='OUT',
        help="Path to the output Tensorflow model (.pb)")
    parser.add_argument(
        "--output_node_name",
        action='append',
        metavar='OUTPUT_NODE_NAME',
        help="Output node name. This parameter must be provided.")
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_arguments()
    main(args)
