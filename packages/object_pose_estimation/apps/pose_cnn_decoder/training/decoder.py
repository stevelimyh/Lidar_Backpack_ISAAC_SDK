'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import lazy_property
import numpy as np
import tensorflow as tf

class Decoder(object):
    """
    Returns segmentation decoded segmentation mask of input color image to encoder

    This class takes outputs from rotation and translation regression networks and is trained
    to learn and output the segmentation image of the input cropped color image. It learns to
    decode the segmentation image of object from pose and is robust to partial occlusions of
    the objects.

    The decoder architecture is based on the work by Sundermeyer et al. with
    source code available at https://github.com/DLR-RM/AugmentedAutoencoder.git under MIT license.
    The architecture is modified to output segmentation image instead of color image and
    conv2d_transpose is used to upscale the image instead of nearest neighbor.

    """

    def __init__(self, reconstruction_target, latent_code, num_filters,
                 kernel_size, strides, loss, bootstrap_ratio,
                 auxiliary_mask, batch_norm, is_training=False):
        """
        Constructor to decoder class
        Initializes decoder class variables
        """
        self._reconstruction_target = reconstruction_target
        self._latent_code = latent_code
        self._auxiliary_mask = auxiliary_mask
        if self._auxiliary_mask:
            self._xmask = None
        self._num_filters = num_filters
        self._kernel_size = kernel_size
        self._strides = strides
        self._loss = loss
        self._bootstrap_ratio = bootstrap_ratio
        self._batch_normalization = batch_norm
        self._is_training = is_training
        self._reconstr_loss = self.reconstr_loss

    @property
    def reconstruction_target(self):
        return self._reconstruction_target

    @lazy_property.LazyProperty
    def x(self):
        """
        Returns decoder segmentation image output
        CNN layers to decode the latent vector to segmentation image
        """
        z = self._latent_code

        h, w, c = self._reconstruction_target.get_shape().as_list()[1:]
        layer_dimensions = [ [int(h/np.prod(self._strides[i:])),
            int(w/np.prod(self._strides[i:]))]  for i in range(len(self._strides))]
        x = tf.layers.dense(
            inputs = self._latent_code,
            units = layer_dimensions[0][0]*layer_dimensions[0][1]*self._num_filters[0],
            activation = tf.nn.relu,
            kernel_initializer = tf.contrib.layers.xavier_initializer()
        )
        if self._batch_normalization:
            x = tf.layers.batch_normalization(x, training=self._is_training)
        x = tf.reshape(x, [-1, layer_dimensions[0][0], layer_dimensions[0][1], self._num_filters[0] ])

        for filters, layer_size in zip(self._num_filters[1:], layer_dimensions[1:]):
            x = tf.layers.conv2d_transpose(
                inputs = x,
                filters = x.get_shape().as_list()[3],
                kernel_size = 3,
                strides = 2,
                padding = 'same'
            )

            x = tf.layers.conv2d(
                inputs = x,
                filters = filters,
                kernel_size = self._kernel_size,
                padding = 'same',
                kernel_initializer = tf.contrib.layers.xavier_initializer_conv2d(),
                activation = tf.nn.relu
            )
            if self._batch_normalization:
                x = tf.layers.batch_normalization(x, training=self._is_training)

        x = tf.layers.conv2d_transpose(
            inputs = x,
            filters = x.get_shape().as_list()[3],
            kernel_size = 3,
            strides = 2,
            padding = 'same'
        )

        if self._auxiliary_mask:
            self._xmask = tf.layers.conv2d(
                    inputs = x,
                    filters = 1,
                    kernel_size = self._kernel_size,
                    padding = 'same',
                    kernel_initializer = tf.contrib.layers.xavier_initializer_conv2d(),
                    activation = tf.nn.sigmoid
            )

        x = tf.layers.conv2d(
                inputs = x,
                filters = c,
                kernel_size = self._kernel_size,
                padding = 'same',
                kernel_initializer = tf.contrib.layers.xavier_initializer_conv2d(),
                activation = tf.nn.sigmoid
        )
        x = tf.identity(x, 'decoder_output')
        return x

    @property
    def reconstr_loss(self):
        """
        Returns decoder reconstruction loss
        Computes the reconstruction loss of the segmentation image with three options
        L1 loss, L2 loss and cross_entropy loss
        """
        tf.summary.image('decoder_output', self.x, max_outputs=1)
        reconstruction_target_flat = tf.contrib.layers.flatten(self._reconstruction_target)
        xmask_flat = tf.contrib.layers.flatten(self._xmask)
        x_flat = tf.contrib.layers.flatten(self.x)
        if self._loss == 'L2':
            if self._bootstrap_ratio > 1:
                l2 = tf.losses.mean_squared_error (
                    reconstruction_target_flat,
                    x_flat,
                    reduction=tf.losses.Reduction.NONE
                )
                l2_val,_ = tf.nn.top_k(l2,k=l2.shape[1]//self._bootstrap_ratio)
                loss = tf.reduce_mean(l2_val)
            else:
                loss = tf.losses.mean_squared_error (
                    self._reconstruction_target,
                    self.x,
                    reduction=tf.losses.Reduction.MEAN
                )
        elif self._loss == 'L1':
            if self._bootstrap_ratio > 1:
                l1 = tf.losses.absolute_difference(
                    reconstruction_target_flat,
                    xmask_flat,
                    reduction=tf.losses.Reduction.NONE
                )
                l1_val,_ = tf.nn.top_k(l1,k=l1.shape[1]/self._bootstrap_ratio)
                loss = tf.reduce_mean(l1_val)
            else:
                l1 = tf.losses.absolute_difference(
                    reconstruction_target_flat,
                    xmask_flat,
                    reduction=tf.losses.Reduction.MEAN
                )
        elif self._loss == 'cross_entropy':
            if self._bootstrap_ratio > 1:
                cross_entropy = tf.compat.v1.losses.sigmoid_cross_entropy (
                    reconstruction_target_flat,
                    x_flat,
                    reduction=tf.losses.Reduction.NONE
                )
                loss = tf.reduce_mean(cross_entropy)
            else:
                loss = tf.compat.v1.losses.sigmoid_cross_entropy_with_logits (
                    reconstruction_target_flat,
                    x_flat,
                    reduction=tf.losses.Reduction.MEAN
                )
        else:
            print('ERROR: UNKNOWN LOSS ', self._loss)
            exit()

        tf.summary.scalar('reconstruction_loss', loss)
        if self._auxiliary_mask:
            mask_loss = tf.losses.mean_squared_error (
                tf.cast(tf.greater(tf.reduce_sum(self._reconstruction_target,
                axis=3,keepdims=True),0.0001),tf.float32),
                self._xmask,
                reduction=tf.losses.Reduction.MEAN
            )
            loss += mask_loss
            tf.summary.scalar('mask_loss', mask_loss)

        return loss