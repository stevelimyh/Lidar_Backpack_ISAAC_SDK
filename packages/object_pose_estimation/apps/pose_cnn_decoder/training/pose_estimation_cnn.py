'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import tensorflow as tf

class PoseEstimationCNN(object):
    """
    Returns total weighted loss from the full 3D pose estimation model for training

    This class builds the full 3D pose estimation model used for training and computes
    the total loss from multiple outputs - translation, rotation, decoder reconstruction losses
    required for the training of the network.
    """

    def __init__(self, encoder, encoder_bbox, regression_translation, regression_rotation, decoder,
                norm_regularize, losses_weights):
        """
        Constructor to Pose_Estimation_CNN class
        Initializes pose estimation cnn class variables needed to build the full model.
        """
        self._encoder = encoder
        self._encoder_bbox = encoder_bbox
        self._regression_translation = regression_translation
        self._regression_rotation = regression_rotation
        self._decoder = decoder
        self._norm_regularize = norm_regularize
        self._losses_weights = losses_weights
        self._loss = self.loss
        tf.compat.v1.summary.scalar('total_loss', self._loss)
        self._global_step = self.global_step

    @property
    def reconstruction(self):
        # Returns the reconstructed segmentation image by the decoder class
        return self._decoder.x

    @property
    def reconstruction_target(self):
        # Ground truth segmentation image to the decoder output
        return self._decoder.reconstruction_target

    @property
    def global_step(self):
        return tf.Variable(0, dtype=tf.int64, trainable=False, name='global_step')

    @property
    def loss(self):
        """
        Returns total loss of the network
        Computes the total loss from multiple components of the network - encoder, decoder,
        translation and rotation regression layers. The full 3D pose estimation model is
        trained to minimize this total weighted loss.
        """
        decoder_loss = self._decoder._reconstr_loss
        if self._norm_regularize > 0:
            decoder_loss += self._encoder.reg_loss * tf.constant(
                self._norm_regularize, dtype = tf.float32)
            decoder_loss += self._encoder_bbox.reg_loss * tf.constant(
                self._norm_regularize, dtype = tf.float32)
            tf.summary.scalar('reg_loss', self._encoder.reg_loss)

        loss = self._losses_weights[0] * self._regression_translation.translation_loss
        loss += self._losses_weights[1] * self._regression_rotation.rotation_loss
        loss += self._losses_weights[2] * decoder_loss

        return loss



