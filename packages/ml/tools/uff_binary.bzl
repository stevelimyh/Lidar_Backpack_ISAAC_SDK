"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

def uff_binary(name, srcs, outs, input_node_names, output_node_names, visibility = None, **kwargs):
    """
    Creates TensorRT model from a TensorFlow Frozen Graph, SavedModel or a Keras model.

    Parameters:
        srcs (list of strings): First element of srcs should contain a path to the input model.
        outs (list of strings): First element of outs should contain a path to the output model.
        input_node_names (list of strings): List of input graph node names, for example ['input']
        output_node_names (list of strings): List of output graph node names, for example ['output']
    """
    native.genrule(
        name = name,
        srcs = srcs,
        outs = outs,
        cmd = "python3 $(execpath //packages/ml/tools:tensorflow_to_tensorrt_tool) $< --out $@ " +
              " ".join(["--input_node_name=%s" % node_name for node_name in input_node_names] +
                       ["--output_node_name=%s" % node_name for node_name in output_node_names]),
        # Run the tool on all files from `srcs` and write the result to the file in `outs`
        message = "Creating TensorRT %s model from %s model." % (outs, srcs),
        tools = ["//packages/ml/tools:tensorflow_to_tensorrt_tool"],
        visibility = visibility,
        **kwargs
    )
