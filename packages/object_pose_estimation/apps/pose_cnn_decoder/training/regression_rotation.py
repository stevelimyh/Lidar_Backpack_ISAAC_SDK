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
import tensorflow_graphics as tfg

class RegressionRotation(object):
    """
    Returns rotation prediction of the object as quaternion

    This class takes the fused latent vector which is encoding of input image and
    bounding parameters and builds regression network that is trained to output and
    learn the rotation of the object as quaternion

    """

    def __init__(self, latent_code, input_rotation, num_fc_layers_rotation,
                loss, batch_norm, quaternion_mag_loss_weight, is_training=False):
        """
        Constructor to Regression_Rotation class
        Initializes regression_rotation class variables needed to build the full model.
        """
        self._latent_code = latent_code
        self._input_rotation = input_rotation
        self._num_fc_layers_rotation = num_fc_layers_rotation
        self._loss = loss
        self._quaternion_mag_loss_weight = quaternion_mag_loss_weight
        self._batch_normalization = batch_norm
        self._is_training = is_training
        self._rotation_prediction = self.rotation_prediction

    @property
    def output(self):
        """
        Outputs the predicted rotation of the object
        """
        return self._rotation_prediction

    @property
    def rotation_prediction(self):
        """
        Returns the rotation prediction of the network as quaternion.
        Series of fully connected layers to predict the rotation from the
        fused latent vector which is output from the encoder_bbox class.
        """
        rotation_prediction = tf.identity(self._latent_code, 'rotation_input')
        for num_fc_layer_units in self._num_fc_layers_rotation:
            rotation_prediction = tf.layers.dense(
                    inputs = rotation_prediction,
                    units = num_fc_layer_units,
                    activation = None,
                    kernel_initializer = tf.contrib.layers.xavier_initializer()
                )
            rotation_prediction = tf.nn.leaky_relu(rotation_prediction, alpha=0.01)
            tf.summary.histogram("rotation layer1", rotation_prediction)

        # Orientation prediction as quaternion {q.w, q.x, q.y, q.z}
        rotation_prediction = tf.layers.dense(
                inputs = rotation_prediction,
                units = 4,
                activation = tf.math.tanh,
                name = "rotation_output",
                kernel_initializer = tf.contrib.layers.xavier_initializer()
            )

        rotation_prediction = tf.identity(rotation_prediction, 'rotation_output')
        return rotation_prediction

    def quaternion_acos_loss(self, i_symm):
        """
        Returns rotation loss based on the angle between two quaternions.
        Args: rotation symmetry index to get the correct ground truth rotation vector.
        Computes the rotation loss based on the angle between predicted and ground truth
        unit quaternions in the 3D space.
        """
        # Quaternion norm square
        quat_sum_squares = tf.math.reduce_sum(
            tf.math.square(self._rotation_prediction),
            axis = 1)
        epsilon_norm_mag = tf.math.scalar_mul(1e-5, tf.ones_like(quat_sum_squares))
        # Making sure norm is non-zero
        quat_sum_squares = tf.math.maximum(quat_sum_squares, epsilon_norm_mag)

        # Input is batch_size x 4
        # Output shape must be batch_size x 1 after this
        quaternions_dot = tf.math.reduce_sum(
            tf.multiply(self._input_rotation[:,i_symm], self._rotation_prediction),
            axis = 1
        )

        quaternions_dot = tf.math.divide(
            tf.math.scalar_mul(2.0, tf.math.square(quaternions_dot)),
            quat_sum_squares
        )
        cos_rotation_value = tf.subtract(quaternions_dot, tf.ones_like(quaternions_dot))
        # Clipping to avoid Nan from acos function (gradient also goes to inf at {-1, 1})
        cos_value_clip = tf.clip_by_value(
            cos_rotation_value,
            clip_value_min = -1.0 + 1e-5,
            clip_value_max = 1.0 - 1e-5
        )
        # Output must be batch_size x 1 after this
        quat_loss_batch = tf.math.abs(tf.math.acos(
            cos_value_clip,
            name = "quat_arc_coss_batch_loss"
        ))
        return quat_loss_batch

    def points_rotation_loss(self, i_symm):
        """
        Returns points translation error based rotation loss called PLOSS
        Args: rotation symmetry index to get the correct ground truth rotation vector.
        The loss function is based on the definition in PoseCNN work:
        https://arxiv.org/abs/1711.00199. The body points are by default taken as the
        vertices of a unit square for now.
        """

        rotation_matrix_prediction = tfg.geometry.transformation.rotation_matrix_3d.from_quaternion(
            self._rotation_prediction
        )
        rotation_matrix_gt = tfg.geometry.transformation.rotation_matrix_3d.from_quaternion(
            self._input_rotation[:, i_symm]
        )
        rotation_matrix_diff = tf.subtract(
            rotation_matrix_prediction,
            rotation_matrix_gt,
        )

        # Body points from unit cube
        body_points = tf.expand_dims(tf.constant([[0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5],
            [-0.5, -0.5, -0.5, -0.5, 0.5, 0.5, 0.5, 0.5],
            [0.5, 0.5, -0.5, -0.5, 0.5, 0.5, -0.5, -0.5]]), 0)
        # Tile it batch_size times - output must be (None, 3, m)
        body_points = tf.tile(body_points, [tf.shape(self._input_rotation)[0], 1, 1])
        points_rotation_matrix_loss = tf.matmul(rotation_matrix_diff, body_points)
        points_rotation_loss_square = tf.reduce_sum(tf.square(points_rotation_matrix_loss), axis=1)
        points_rotation_loss = 0.5*tf.reduce_mean(points_rotation_loss_square, axis=1)
        return points_rotation_loss

    @lazy_property.LazyProperty
    def rotation_loss(self):
        """
        Returns rotation loss of the network
        Currently, two different loss functions are supported to compute the rotation loss
        1. AcosLoss: Loss based on angles between quaternions of prediction and ground truth.
        2. PLoss: based on the rotated point coordinates of the body points
        Object rotation symmetries are handled in these loss functions by considering the
        minimum of loss over all the symmetric ground truth rotations as the final rotation loss.
        """
        quat_sum_squares = tf.math.reduce_sum(
            tf.math.square(self._rotation_prediction),
            axis = 1)

        # Loss to force the output quaternion to be unit quaternion
        quaternion_magnitude_loss = tf.compat.v1.losses.absolute_difference(
            tf.ones_like(quat_sum_squares),
            quat_sum_squares
        )

        # To handle object symmetry, find minimum over rotation losses of all ground
        # truth symmetric rotations.
        # Loss function is acos(2*dot(q_gt, q_prediction)^2 - 1)
        if (self._loss == 'AcosLoss'):
            rotation_quat_loss = self.quaternion_acos_loss(0)
        elif (self._loss == 'PLoss'):
            rotation_quat_loss = self.points_rotation_loss(0)
        else:
            print('ERROR: UNKNOWN ROTATION LOSS ', self._loss)
            exit()

        for i_symm in range(1, self._input_rotation.shape[1]):
            if (self._loss == 'AcosLoss'):
                quaternion_symm_loss = self.quaternion_acos_loss(i_symm)
            elif (self._loss == 'PLoss'):
                quaternion_symm_loss = self.points_rotation_loss(i_symm)
            else:
                print('ERROR: UNKNOWN ROTATION LOSS ', self._loss)
                exit()

            rotation_quat_loss = tf.math.minimum(rotation_quat_loss, quaternion_symm_loss)
        rotation_quat_loss = tf.math.reduce_mean(rotation_quat_loss)

        tf.compat.v1.summary.scalar('quaternion_mag_loss', quaternion_magnitude_loss)
        tf.compat.v1.summary.scalar('rotation_quat_loss', rotation_quat_loss)
        rotation_loss = self._quaternion_mag_loss_weight * quaternion_magnitude_loss
        rotation_loss += rotation_quat_loss
        tf.compat.v1.summary.scalar('rotation_loss', rotation_loss)
        return rotation_loss
