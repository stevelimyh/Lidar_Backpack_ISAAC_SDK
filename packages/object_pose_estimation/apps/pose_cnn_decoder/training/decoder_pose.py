'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import tensorflow as tf

class DecoderPose(object):
    """
    Returns fused latent vector from the pose predicted by regression layers in the model.

    This class takes outputs from rotation and translation regression networks, encodes and fuse
    the latent vectors. The output from this model is then used to decode the object's segmentation
    mask by decoder class.
    """

    def __init__(self, translation, rotation,
                 latent_space_size, is_training=True):
        """
        Constructor to Decoder_Pose class
        Initializes decoder_pose class variables
        """
        self._translation = translation
        self._rotation = rotation
        self._latent_space_size = latent_space_size
        self._is_training = is_training
        self._decoder_pose = self.decoder_pose

    @property
    def decoder_pose(self):
        """
        Returns latent vector from rotation and translation parameters
        Fully connected layers to encode rotation and translation parameters
        and fuses them.
        """
        translation_decoder= tf.layers.dense(
                inputs = self._translation,
                units = self._latent_space_size,
                activation = None,
                kernel_initializer = tf.contrib.layers.xavier_initializer()
        )
        translation_decoder = tf.nn.leaky_relu(translation_decoder, alpha=0.01)

        rotation_decoder= tf.layers.dense(
                inputs = self._rotation,
                units = self._latent_space_size,
                activation = None,
                kernel_initializer = tf.contrib.layers.xavier_initializer()
        )
        rotation_decoder = tf.nn.leaky_relu(rotation_decoder, alpha=0.01)

        # Concatenates the translation and rotation decoder
        decoder_pose = tf.concat(
                values = [rotation_decoder, translation_decoder],
                axis = 1
        )

        decoder_pose = tf.layers.dense(
                inputs = decoder_pose,
                units = 2*self._latent_space_size,
                activation = None,
                kernel_initializer = tf.contrib.layers.xavier_initializer()
        )
        decoder_pose = tf.nn.leaky_relu(decoder_pose, alpha=0.01)
        return decoder_pose
