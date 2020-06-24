'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

'''
This code file was inspired by Spinning up repository at
https://github.com/openai/spinningup
under the following license:

MIT License

Copyright (c) 2018 OpenAI (http://openai.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import numpy as np
import tensorflow.compat.v1 as tf


def get_trainable_vars(scope):
    """Returns the trainable variables within the passed scope"""
    return [var for var in tf.global_variables() if scope in var.name]


def clip_but_pass_gradient(x, low, high):
    """Clips tensor but passes the gradient of the tensor"""
    clip_high = tf.cast(x > high, tf.float32)
    clip_low = tf.cast(x < low, tf.float32)
    return x + tf.stop_gradient((high - x) * clip_high + (low - x) * clip_low)


def apply_squashing_function(mu, pi, logp_pi):
    """Apply squashing function and clip"""
    mu = tf.tanh(mu)
    pi = tf.tanh(pi)
    # Clip 1-pi**2 to [0,1] range with small epsilon.
    logp_pi -= tf.reduce_sum(
        tf.log(clip_but_pass_gradient(1 - pi**2, low=0, high=1) + 1e-6), axis=1)
    return mu, pi, logp_pi

def policy_backbone(pose,
                    observation_map,
                    concat_policy=None,
                    output_dimension=None,
                    conv_filters=(16, 32),
                    dense_units=(256,256),
                    kernel_width=3,
                    strides=1,
                    pooling_width=2,
                    pooling_strides=2,
                    hidden_activation='relu',
                    output_activation=None,
                    dropout=0.0):
    """Create the base neural network for the policy and Q functions
    pose, observation_map : Input tensors consisting of pose and occupancy grid
    concat_policy : Tensor that needs to be appended to the input feature vector
    output_dimension : length of output of the neural network
    conv_filters : number of filters for each Convolutional layer
    dense_units : dimension of fully connected layers
    kernel_width : width of the kernels for convolution
    strides : stride of convolutional neural network
    pooling_width : size of pooling filter
    hidden activation : activation function used in the backbone
    output_activation : activation of the output layer
    dropout : Dropout values in the convolutional network
    """
    num_conv_layers = len(conv_filters)
    num_dense_layers = len(dense_units)
    num_hidden_layers = num_conv_layers + num_dense_layers
    num_layers = num_hidden_layers + 1
    pooling_width = (pooling_width,) * num_conv_layers
    pooling_strides = (pooling_strides,) * num_conv_layers
    hidden_activation_array = (hidden_activation,) * num_hidden_layers
    dropout = (dropout,) * num_layers
    initializer = tf.initializers.variance_scaling(scale=2.0)
    # Convolutional part of the neural network
    for i in range(num_conv_layers):
        observation_map = tf.layers.conv2d(inputs=observation_map,
                             filters=conv_filters[i],
                             kernel_size=(kernel_width, kernel_width),
                             strides=(strides, strides),
                             activation=hidden_activation_array[i],
                             kernel_initializer=initializer,
                             padding='SAME')
        observation_map = tf.layers.batch_normalization(inputs=observation_map)
        observation_map = tf.layers.max_pooling2d(inputs=observation_map,
                                    pool_size=(pooling_width[i], pooling_width[i]),
                                    strides=(pooling_strides[i], pooling_strides[i]))
    # Flatten the 2D feature map to 1D feature map
    observation_map = tf.layers.flatten(inputs=observation_map)
    observation_map = tf.layers.dense(inputs=observation_map,
                    units=conv_filters[-1]*conv_filters[-1],
                    activation=hidden_activation,
                    kernel_initializer=initializer)
    # Append the observation map feature vector with pose feature vector
    x = tf.concat([observation_map, pose], axis=-1)
    # Concat the action or policy into the network just before fully connected layers
    if concat_policy is not None:
        x = tf.concat([x,concat_policy], axis=-1)

    # Create the dense layers
    for i, j in enumerate(range(num_conv_layers, num_hidden_layers)):
        x = tf.layers.dense(inputs=x,
                            units=dense_units[i],
                            activation=hidden_activation_array[j],
                            kernel_initializer=initializer)
        if dropout[j] > 0.0:
            x = tf.layers.dropout(inputs=x,
                                  rates=dropout[j])
    # Create output layer
    if output_dimension is not None:
        x = tf.layers.dense(inputs=x,
                            units=output_dimension,
                            activation=output_activation,
                            kernel_initializer=initializer)
    return x

def generate_gaussian_policy(pose, observation_map, action_tensor, output_activation):
    """Create the policy neural network
    The parameters accepted are :
    pose, observation_map : Input tensors consisting of pose and occupancy grid
    action_tensor : Action placeholder tensor
    output_activation : Activation function for output of neural network
    """
    LOG_STD_MAX = 2
    LOG_STD_MIN = -20
    action_dimension = action_tensor.shape.as_list()[-1]

    net = policy_backbone(pose, observation_map)

    # mu and log_std branch from the network in parallel
    mu = tf.layers.dense(net, action_dimension, activation=output_activation)
    log_std = tf.layers.dense(net, action_dimension, activation=tf.tanh)
    log_std = LOG_STD_MIN + 0.5 * (LOG_STD_MAX - LOG_STD_MIN) * (log_std + 1)
    std = tf.exp(log_std)
    pi = mu + tf.random_normal(tf.shape(mu)) * std

    # Compute gaussian likelihood
    pre_sum = -0.5 * (((pi - mu) / (tf.exp(log_std) + (1e-8)))**2 + 2 * log_std + np.log(2 * np.pi))
    logp_pi = tf.reduce_sum(pre_sum, axis=1)
    return mu, pi, logp_pi


def soft_actor_critic_architecture(pose,
                                   observation_map,
                                   action_tensor,
                                   action_scale,
                                   activation=tf.nn.relu,
                                   output_activation=None):
    """Creates Soft-Actor-Critic training infrastructure.
    It consists of 2 Q-neural networks, their corresponding target/backup
    neural networks and a value function network. All these collectively form
    the Soft Actor Critic architecture
    The parameters accepted are :
    pose, observation_map : Input tensors consisting of pose and occupancy grid
    action_tensor : Action placeholder tensor
    action_scale : Scale of ranges for output action
    activation : Activation function for each hidden layer
    output_activation : Activation function for output of neural network
    """
    # Establish policy function
    with tf.variable_scope('pi'):
        mu, pi, logp_pi = generate_gaussian_policy(pose, observation_map, action_tensor,
                                                   output_activation)
        mu, pi, logp_pi = apply_squashing_function(mu, pi, logp_pi)

    # Normalize actions in correct range
    mu *= action_scale
    pi *= action_scale

    # Create all the neural networks for Q & V functions respectively
    base_network = lambda pose_param, observation_map_param, concat_policy_param : tf.squeeze(
                            policy_backbone(pose = pose_param,
                                            observation_map = observation_map_param,
                                            concat_policy = concat_policy_param,
                                            output_dimension=1), axis=1)
    # Q1-Network
    with tf.variable_scope('q1'):
        q1 = base_network(pose, observation_map, action_tensor)
    # Q1 backup network
    with tf.variable_scope('q1', reuse=True):
        q1_pi = base_network(pose, observation_map, pi)
    # Q2-Network
    with tf.variable_scope('q2'):
        q2 = base_network(pose, observation_map, action_tensor)
    # Q2 backup network
    with tf.variable_scope('q2', reuse=True):
        q2_pi = base_network(pose, observation_map, pi)
    return mu, pi, logp_pi, q1, q2, q1_pi, q2_pi
