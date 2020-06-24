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

class RegressionTranslation(object):
    """
    Returns predictions of object center in image coordinates and depth in real world.

    This class takes the fused latent vector which is encoding of input image and
    bounding parameters and builds regression network that is trained to output and
    learn the object center in image coordinates and depth in real world.

    """

    def __init__(self, latent_code, input_translation, num_fc_layers_translation,
                 loss, batch_norm, is_training=False):
        """
        Constructor to Regression_Translation class
        Initializes regression_translation class variables needed to build the full model.
        """
        self._latent_code = latent_code
        self._input_translation = input_translation
        self._num_fc_layers_translation = num_fc_layers_translation
        self._loss = loss
        self._batch_normalization = batch_norm
        self._is_training = is_training
        self._translation_prediction = self.translation_prediction

    @property
    def output(self):
        # Returns the translation prediction from the network
        return self._translation_prediction

    @property
    def translation_prediction(self):
        """
        Returns the translation parameters prediction of the network.
        Series of fully connected layers to predict the translation parameters
        of the network in order - object_center.x, object_center.y, object_depth
        from used latent vector which is output from the encoder_bbox class.
        """
        translation_prediction = tf.identity(self._latent_code, 'translation_input')
        for num_fc_layer_units in self._num_fc_layers_translation:
            translation_prediction = tf.layers.dense(
                    inputs = translation_prediction,
                    units = num_fc_layer_units,
                    activation = None,
                    kernel_initializer = tf.contrib.layers.xavier_initializer()
                )
            translation_prediction = tf.nn.leaky_relu(translation_prediction, alpha=0.01)
        # Translation prediction Tx, Ty and Tz(depth) - linear activation function
        translation_prediction = tf.layers.dense(
                inputs = translation_prediction,
                units = 3,
                activation = None,
                kernel_initializer = tf.contrib.layers.xavier_initializer()
            )
        translation_prediction = tf.identity(translation_prediction, 'translation_output')
        return translation_prediction

    @lazy_property.LazyProperty
    def translation_loss(self):
        """
        Returns tanslation loss that is weighted sum of center and depth losses of object.
        Computes weighted sum of L1 loss of the predictions of object center and depth.
        """
        translation_offset_loss = tf.compat.v1.losses.absolute_difference(
            self._input_translation[:, :2],
            self._translation_prediction[:, :2],
            reduction=tf.compat.v1.losses.Reduction.MEAN)
        translation_depth_loss = tf.compat.v1.losses.absolute_difference(
            self._input_translation[:, 2],
            self._translation_prediction[:, 2],
            reduction=tf.compat.v1.losses.Reduction.MEAN
        )
        tf.compat.v1.summary.scalar('translation_offset_loss', translation_offset_loss)
        tf.compat.v1.summary.scalar('translation_depth_loss', translation_depth_loss)
        translation_loss = translation_offset_loss + 0.5*translation_depth_loss
        return translation_loss
