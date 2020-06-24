'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import lazy_property
import tensorflow as tf

class Encoder(object):
    """
    Returns encoding/latent vector of input color image to the network.

    This class takes input cropped color image of the object and outputs the
    encoding of the image as a vector. This latent vector is sent as input to
    Encoder_Bbox class to fuse it with bounding box parameter encoding before
    sending it to the decoders.

    The encoder architecture is based on the work by Sundermeyer et al. with
    source code available at https://github.com/DLR-RM/AugmentedAutoencoder.git
    under MIT license.
    """

    def __init__(self, input, latent_space_size, num_filters, kernel_size, strides, batch_norm,
                 is_training=False):
        """
        Constructor to Encoder class
        Initializes encoder class variables
        """
        self._input = input
        self._latent_space_size = latent_space_size
        self._num_filters = num_filters
        self._kernel_size = kernel_size
        self._strides = strides
        self._batch_normalization = batch_norm
        self._is_training = is_training
        self._encoder_out = self.encoder_out
        self._z = self.z

    @property
    def x(self):
        return self._input

    @property
    def latent_space_size(self):
        return self._latent_space_size

    @property
    def encoder_out(self):
        """
        Returns flattened output after downsampling the input image
        Series of convolutional layers to encode the input color image.
        """
        x = self._input
        x = tf.identity(x, 'encoder_input')

        for filters, stride in zip(self._num_filters, self._strides):
            padding = 'same'
            x = tf.layers.conv2d(
                inputs = x,
                filters = filters,
                kernel_size = self._kernel_size,
                strides = stride,
                padding = padding,
                kernel_initializer = tf.contrib.layers.xavier_initializer_conv2d(),
                activation = tf.nn.relu
            )
            if self._batch_normalization:
                x = tf.layers.batch_normalization(x, training = self._is_training)

        h, w, c = x.get_shape().as_list()[1:]
        encoder_out = tf.reshape(x, [-1, h*w*c])
        return encoder_out

    @property
    def z(self):
        """
        Returns encoding of the input color image
        Fully connected layer to encode the output from CNN based encoder network
        """
        x = self._encoder_out
        z = tf.layers.dense(
            x,
            self._latent_space_size,
            activation = None,
            name = 'encoder_output',
            kernel_initializer = tf.contrib.layers.xavier_initializer()
        )
        return z

    @lazy_property.LazyProperty
    def reg_loss(self):
        """
        Returns regularization loss
        Ensures that the weights are bounded and that the trained model is not over-fitted.
        """
        reg_loss = tf.reduce_mean(tf.abs(tf.norm(self._z, axis = 1) - tf.constant(1.)))
        return reg_loss
