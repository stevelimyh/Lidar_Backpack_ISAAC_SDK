'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
"""
Isaac TensorFlow to TensorRT conversion tool.

Converts a neural network in the Frozen Graph (.pb), Keras Model (.hdf5) or a Saved Model (./)
format into the TensorRT (.uff) model.

usage: tensorflow_to_tensorrt.py [-h] [--out OUT]
                                 [--input_node_name INPUT_NODE_NAME]
                                 [--output_node_name OUTPUT_NODE_NAME]
                                 INPUT_MODEL_PATH

positional arguments:
  INPUT_MODEL_PATH      Path to the input TensorFlow model (.pb|.hdf5|./)

optional arguments:
  -h, --help            show this help message and exit
  --out OUT             Path to the output TensorRT model (.uff)
  --input_node_name INPUT_NODE_NAME
                        Input node name. This parameter could be repeated.
  --output_node_name OUTPUT_NODE_NAME
                        Output node name. This parameter could be repeated.
"""

import argparse, sys, os.path, uff
import tensorflow as tf
from tensorflow import keras as K
from tensorflow.python.tools import optimize_for_inference_lib

def main(args):
    """
    Loads a TensorFlow Frozen Graph (.pb), a Keras Model (.hdf5) or a Saved Model folder
    (loads, removes training nodes, optimizes for inference, converts to and saves TensorRT model).

    Arguments:
    args: the parsed command line arguments
    """

    # load the model from args.input_model_path into graph_def
    if args.input_model_path.endswith(".pb"):
        graph_def = tf.GraphDef()
        with open(args.input_model_path, 'rb') as f:
            graph_def.ParseFromString(f.read())
    elif args.input_model_path.endswith(".hdf5") or args.input_model_path.endswith(".h5"):
        K.backend.set_learning_phase(0)
        model = K.models.load_model(SAVED_MODEL_FP)
        session = K.backend.get_session()
        graph_def = session.graph.as_graph_def()
    else:
        with tf.Session(graph=tf.Graph()) as session:
            tf.saved_model.loader.load(
                session, [tf.saved_model.tag_constants.SERVING],
                args.input_model_path,
                strip_default_attrs=True)
            graph_def = session.graph.as_graph_def()

    if not args.out:
        args.out = os.path.splitext(args.input_model_path)[0] + ".uff"

    # attempt to deduce input nodes
    input_node_names, output_node_names = args.input_node_name, args.output_node_name
    if not args.input_node_name:
        print("No input node names provided, assuming (input.*) search pattern in the graph...")
        input_node_names = [n.name for n in graph_def.node if n.name.startswith("input")]

    if not args.output_node_name:
        print("No output node names provided, assuming (output.*) search pattern in the graph...")
        output_node_names = [n.name for n in graph_def.node if n.name.startswith("output")]

    print("Using %s as input nodes, %s as output nodes." % (input_node_names, output_node_names))
    if not input_node_names or not output_node_names:
        sys.exit('Failed to identify input or output nodes in the graph. Exiting.')

    # attempt to optimize graph for inference (rm placeholder, etc)
    graph_def = optimize_for_inference_lib.optimize_for_inference(
        graph_def, input_node_names, output_node_names, tf.float32.as_datatype_enum)

    # convert variables to constants
    with tf.Session() as session:
        graph_def = tf.graph_util.convert_variables_to_constants(session, graph_def,
                                                                 output_node_names)

    # Convert inference graph to UFF
    uff.from_tensorflow(graph_def, output_node_names, output_filename=args.out)


def parse_arguments():
    """
    Set required input arguments and add functionality to parse them

    Returns:
    The arguments parsed from the command line
    """
    parser = argparse.ArgumentParser(
        description="Isaac TensorFlow to TensorRT conversion tool.\n"
        "Converts a neural network in Frozen Graph (.pb), Keras Model (.hdf5) or "
        "Saved Model (./) formats into the TensorRT (.uff) model.)")
    parser.add_argument(
        "input_model_path",
        metavar="INPUT_MODEL_PATH",
        help="Path to the input TensorFlow model (.pb|.hdf5|./)")
    parser.add_argument("--out", help="Path to the output TensorRT model (.uff)")
    parser.add_argument(
        "--input_node_name",
        action='append',
        metavar='INPUT_NODE_NAME',
        help="Input node name. This parameter could be repeated.")
    parser.add_argument(
        "--output_node_name",
        action='append',
        metavar='OUTPUT_NODE_NAME',
        help="Output node name. This parameter could be repeated.")
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_arguments()
    main(args)
