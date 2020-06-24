from os import listdir
from os.path import isfile, join
from tensorflow.core.framework import graph_pb2
from tf2onnx import loader, optimizer, utils
from tf2onnx.tfonnx import process_tf_graph, tf_optimize

import argparse
import re
import tensorflow as tf

# Name of the iterator node which passes input to the first convolution layer
# While freezing, we go through the graph and replace the iterator nodes with the input node
kIteratorName = "IteratorGetNext"
kInputName = "input"

def main(args):
    """
    Checkpoints are stored as a combination of index, meta and data files
    Freeze the most recent checkpoint into a protobuf file

    Arguments:
    args: the parsed command line arguments
    """
    # Get the file path for the most recent meta
    meta_filename = get_latest_meta(args.checkpoint_dir)
    meta_path = join(args.checkpoint_dir, meta_filename)
    # Get output node name and full path for the output file of the frozen graph
    output_node_names = [args.output_nodename]
    output_file_path = join(args.checkpoint_dir, args.output_filename)
    with tf.Session(config=tf.ConfigProto(allow_soft_placement=True)) as sess:
        # Restore the graph
        saver = tf.train.import_meta_graph(meta_path)
        # Load weights
        saver.restore(sess, tf.train.latest_checkpoint(args.checkpoint_dir))
        # Freeze the graph
        frozen_graph_def = tf.graph_util.convert_variables_to_constants(
            sess, sess.graph_def, output_node_names)
        # Create a graph with an input placeholder,
        with tf.Graph().as_default() as placeholder:
            input = tf.placeholder(tf.float32, name=kInputName, shape=[1, 256, 512, 3])
        # Create a graph definition and add the input placeholder to it
        output_graph_def = graph_pb2.GraphDef()
        output_graph_def.node.extend(placeholder.as_graph_def().node[:1])
        # Extend the new graph definition with all the nodes from the old one except the iterators
        output_graph_def.node.extend(frozen_graph_def.node[1:])
        # Make sure none of the remaining nodes accept input from an iterator node
        for node in output_graph_def.node:
            if kIteratorName in node.input:
                for i in range(len(node.input)):
                    if node.input[i] == kIteratorName:
                        node.input[i] = kInputName
        # Save the frozen graph
        with open(output_file_path, 'wb') as f:
            f.write(output_graph_def.SerializeToString())
    # If an ONNX file path was not provided, stop there
    if not args.output_onnx_filename:
        return
    # If a file path was provided, freeze as ONNX
    onnx_filepath = join(args.checkpoint_dir, args.output_onnx_filename)
    convert_to_onnx(output_file_path, onnx_filepath, output_node_names)


def convert_to_onnx(model_filepath, onnx_filepath, output_node_names):
    """
    Convert the model to an ONNX file, which can in turn be used for TensorRT inference

    Arguments:
    model_filepath: the path to the frozen .pb file
    onnx_filepath: the path where the ONNX file should be saved
    output_node_names: list of output node names
    """
    # tf2onnx expects the node names in the format "input/output_node_name:port_id".
    # Hence, we should provide the port ID before conversion.
    input_node_names = [kInputName + ":0"]
    output_node_names = list(map(lambda x: x + ":0", output_node_names))
    # Use in-built function from tf2onnx to import the graph and optimize for conversion
    graph_def, inputs, outputs = loader.from_graphdef(model_filepath, input_node_names,
                                                      output_node_names)
    graph_def = tf_optimize(input_node_names, output_node_names, graph_def, False)
    with tf.Graph().as_default() as default_graph:
        tf.import_graph_def(graph_def, name='')
    # Convert to ONNX
    with tf.Session(graph=default_graph):
        onnx_graph = process_tf_graph(
            default_graph, opset=8, input_names=inputs, output_names=outputs)
    onnx_graph = optimizer.optimize_graph(onnx_graph)
    onnx_model = onnx_graph.make_model("segmentation_onnx_model")
    # Save the ONNX model to disk
    utils.save_protobuf(onnx_filepath, onnx_model)


def get_latest_meta(checkpoint_dir):
    """
    Find the most recent meta file in the checkpoint directory
    Checkpoints are stored in the form of three files with extensions as follows:
    * .meta: Stores the graph structure (eg.: model.ckpt-100.meta)
    * .data: Stores the values of the variables saved (eg.: model.ckpt-100.data-00000-of-00001)
    * .index: Stores the list of variable names and shapes (eg.: model.ckpt-100.index)
    The meta file is neccessary to build the model graph before loading the saved weights

    Arguments:
    checkpoint_dir: the path to the checkpoint directory

    Returns:
    Filename of the most recently checkpointed meta file
    """
    # List all meta files in checkpoint directory
    meta_files = [
        f for f in listdir(checkpoint_dir) if isfile(join(checkpoint_dir, f)) and "meta" in f
    ]
    # List corresponding step numbers in their respective filenames
    meta_indices = list(map(lambda x: int(re.findall('\d+', x)[0]), meta_files))
    # Find the one with the largest index
    meta_filename = meta_files[meta_indices.index(max(meta_indices))]
    return meta_filename


def parse_arguments():
    """
    Set required input arguments and add functionality to parse them

    Returns:
    The arugments parsed from the command line
    """
    parser = argparse.ArgumentParser(description="Freezes (or exports) a model from TensorFlow \
                                                  checkpoints.")
    required_arguments = parser.add_argument_group('required arguments')
    required_arguments.add_argument("--checkpoint_dir", help="Path to the checkpoint directory. \
                                     Example: /tmp/path_segmentation", required=True)
    required_arguments.add_argument("--output_filename", help="Output file name. \
                                     Example: model.pb", required=True)
    required_arguments.add_argument("--output_nodename", help="Output node name. \
                                     Example: prediction/truediv", required=True)
    parser.add_argument("--output_onnx_filename", help="Filename for the output ONNX model. \
                         Example: model.onnx")
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_arguments()
    main(args)
