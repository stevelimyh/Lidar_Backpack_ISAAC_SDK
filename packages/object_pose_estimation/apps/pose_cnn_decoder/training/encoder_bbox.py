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

class EncoderBbox(object):
    """
    Returns concatenated encodings/latent vector of input image and bounding box parameters.

    This class takes bounding box parameters namely, size and center as input, encodes it into
    a latent vector and fuses with the encoding from input color image. This fused latent vector
    is sent as output that is used to estimate the translation and rotation parameters and
    decode the color image to segmentation mask.
    """

    def __init__(self, input_bbox, encoder_image_latent_code,
                 num_fc_layers_encoder_bbox, batch_norm, is_training=False):
        """
        Constructor to Encoder_Bbox class
        Initializes encoder_bbox class variables
        """
        self._input_bbox = input_bbox
        self._encoder_image_latent_code = encoder_image_latent_code
        self._num_fc_layers_encoder_bbox = num_fc_layers_encoder_bbox
        self._batch_normalization = batch_norm
        self._is_training = is_training
        self._encoder_bbox_out = self.encoder_bbox_out
        self._concat_latent_code = self.concat_latent_code

    @property
    def input(self):
        return self._input_bbox

    @property
    def encoder_bbox_out(self):
        """
        Returns encoding of the bounding box parameters
        Series of fully connected layers to encode the bounding box parameters
        """
        encoder_bbox_out = self._input_bbox
        encoder_bbox_out = tf.identity(encoder_bbox_out, 'encoder_bbox_input')

        for num_fc_layer_units in self._num_fc_layers_encoder_bbox[:]:
            encoder_bbox_out = tf.layers.dense(
                    inputs = encoder_bbox_out,
                    units = num_fc_layer_units,
                    activation = tf.nn.relu,
                    kernel_initializer = tf.contrib.layers.xavier_initializer()
                )
        return encoder_bbox_out

    @property
    def concat_latent_code(self):
        """
        Returns the concatenated latent vectors of input image and bounding box params
        Cancatenates the encodings from both the inputs to the model - cropped color image
        and bounding box parameters (size and center).
        """
        x = self._encoder_bbox_out
        tf.compat.v1.summary.scalar('encoder output mean',
            tf.reduce_mean(self._encoder_image_latent_code))
        tf.compat.v1.summary.scalar('encoder_bbox output mean', tf.reduce_mean(x))
        # Concatenate the output latent code from encoder bbox fc layers and encoder_image
        concat_latent_code = tf.concat(
                values = [self._encoder_image_latent_code, x],
                axis = 1
        )
        # One more fc after concatenation which will be the hidden vector
        concat_latent_code = tf.layers.dense(
                inputs = concat_latent_code,
                units = 2*self._num_fc_layers_encoder_bbox[-1],
                activation = tf.nn.relu,
                kernel_initializer = tf.contrib.layers.xavier_initializer(),
                name = 'encoder_concat_output'
        )
        return concat_latent_code

    @lazy_property.LazyProperty
    def reg_loss(self):
        """
        Returns regularization loss
        Ensures that the weights are bounded and that the trained model is not over-fitted.
        """
        reg_loss = tf.reduce_mean(tf.abs(tf.norm(self._encoder_bbox_out,axis=1) - tf.constant(1.)))
        return reg_loss
