'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import cv2
import numpy as np
import tensorflow as tf

from external.object_pose_estimation_aae.auto_pose.ae.ae import AE
from external.object_pose_estimation_aae.auto_pose.ae.decoder import Decoder
from external.object_pose_estimation_aae.auto_pose.ae.encoder import Encoder

# The functions in this script are slightly modified versions of helper or
# wrapper functions to the autoencoder network obtained from the public
# github source code (https://github.com/DLR-RM/AugmentedAutoencoder.git).

def extract_square_patch(scene_img, bb_xywh, pad_factor=1.0, resize=(128,128),
                         interpolation=cv2.INTER_NEAREST,black_borders=True):
    """
    Extract a square patch around the bounding box with padding and resize
    to network input size
    """

    x, y, w, h= np.array(bb_xywh).astype(np.int32)
    size = int(np.maximum(h, w) * pad_factor)
    left = np.maximum(x+w/2-size/2, 0).astype(np.int)
    right = np.minimum(x+w/2+size/2, scene_img.shape[0]).astype(np.int)
    top = np.maximum(y+h/2-size/2, 0).astype(np.int)
    bottom = np.minimum(y+h/2+size/2, scene_img.shape[1]).astype(np.int)
    scene_crop = scene_img[left:right, top:bottom].copy()
    if black_borders:
        scene_crop[:, :(y-top)] = 0
        scene_crop[:, (y+h-top):] = 0
        scene_crop[:(x-left), :] = 0
        scene_crop[(x+w-left):, :] = 0
    scene_crop = cv2.resize(scene_crop, resize, interpolation = interpolation)
    return scene_crop

# The functions below are overload of the helper functions in the github source code
# to build the autoencoder network and set up the training in tensorflow.

def build_encoder(x, latent_space_size, num_filter, kernel_size_encoder,
    strides, batch_norm, is_training=True):
    # The packaged source code for the architecture is a forked version of the author's github
    # source code (https://github.com/DLR-RM/AugmentedAutoencoder.git) with minor modifications
    # to Encoder class to make the model compatible with TensorRT model. The modifications made
    # are: adding name to the input and output layers, and using tf.reshape() instead of
    # tf.contrib.layers.flatten() for last encoder layer.

    encoder = Encoder(
        x,
        latent_space_size,
        num_filter,
        kernel_size_encoder,
        strides,
        batch_norm,
        is_training=is_training
    )

    return encoder

def build_decoder(reconstruction_target, encoder, variational, num_filter,
    kernel_size_decoder, strides, loss, bootstrap_ratio, auxiliary_mask,
    batch_norm, is_training=True):

    # Modifications to Decoder class include adding name to decoder output variable, saving
    # reconstructed image by decoder to tensorflow summary writer for analysis.
    decoder = Decoder(
        reconstruction_target,
        encoder.sampled_z if variational else encoder.z,
        list( reversed(num_filter) ),
        kernel_size_decoder,
        list( reversed(strides) ),
        loss,
        bootstrap_ratio,
        auxiliary_mask,
        batch_norm,
        is_training=is_training
    )
    return decoder

def build_ae(encoder, decoder, norm_regularize, variational):
    ae = AE(encoder, decoder, norm_regularize, variational)
    return ae

def build_train_op(ae, learning_rate, optimizer_name= "Adam"):

    optimizer = eval('tf.train.{}Optimizer'.format(optimizer_name))
    optim = optimizer(learning_rate)
    train_op = tf.contrib.training.create_train_op(ae.loss, optim, global_step=ae.global_step)

    return train_op
