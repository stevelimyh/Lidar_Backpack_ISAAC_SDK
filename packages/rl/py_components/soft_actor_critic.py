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

from packages.rl.py_components.soft_actor_critic_utils import soft_actor_critic_architecture, get_trainable_vars


class SoftActorCriticPolicy:
    """Create a Soft Actor Critic Policy for Reinforcement Learning.

    A policy dictates what actions an agent should take given a particular state

    The structure of the SAC policy and the specifications are specified in
    Paper : https://arxiv.org/abs/1812.05905
    """

    def __init__(self, size_pose_state, observation_map_dimension, look_back, size_action_space, action_scale,
                 learning_rate, polyak, gamma, log_dir):
        """Initialize the policy and creates the necessary neural networks.
            The parameters needed to create a SAC Policy are :
            size_pose_state : Dimension of pose of the robot at every timestep
            observation_map_dimension : Dimension of the observation map (side of the square)
            size_action_space : Action dimension for each agent
            action_scale : The range of output action values desired
            learning_rate : Learning rate for the neural networks
            polyak : Interpolation between main and target networks
            gamma : Discount factor
            log_dir : Path to store Tensorboard logs
        """
        self.action_dimension_per_agent = size_action_space
        self.size_pose_state = size_pose_state
        self.observation_map_dimension = observation_map_dimension
        self.look_back = look_back
        self.state_dimension_per_agent = size_pose_state + (
            observation_map_dimension * observation_map_dimension)
        self.training_step_count = 0
        # Initializes tensorflow graph
        tf.reset_default_graph()
        self.graph = tf.Graph()
        # Only use the amount of GPU necessary
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # Intialize our TF Session
        self.sess = tf.Session(graph=self.graph, config=config)

        # Create the neural network
        with self.graph.as_default():
            # Reinforcement learning inputs are generally fed as a tuple (s,a,s',r,d)
            self.observations_before = tf.placeholder(
                dtype=tf.float32,
                shape=(None, self.state_dimension_per_agent*self.look_back),
                name='observations_before')
            self.observations_after = tf.placeholder(
                dtype=tf.float32,
                shape=(None, self.state_dimension_per_agent*self.look_back),
                name='observations_after')
            self.action = tf.placeholder(
                dtype=tf.float32, shape=(None, self.action_dimension_per_agent), name='action')
            self.reward = tf.placeholder(dtype=tf.float32, shape=(None, ), name='reward')
            self.death = tf.placeholder(dtype=tf.float32, shape=(None, ), name='death')

            # Extract the pose state and occupancy grid from the aggregated state tensor
            batch_size = tf.shape(self.observations_before)[0]
            self.state_before = tf.reshape(
                self.observations_before,
                [batch_size, self.look_back, self.state_dimension_per_agent])
            # Extract the pose of the agent before the action was performed
            self.pose_state_before = tf.reshape(
                self.state_before[:, :, :self.size_pose_state],
                [batch_size, self.size_pose_state * self.look_back])
            # Extract the occupancy grid of the agent before the action was performed
            extracted_observation_map_before = tf.transpose(
                self.state_before[:, :, self.size_pose_state:], perm=[0, 2, 1])
            self.observation_map_before = tf.reshape(
                extracted_observation_map_before,
                [batch_size, self.observation_map_dimension, self.observation_map_dimension,
                self.look_back])
            # Extract the state of the agent after the action was performed
            self.state_after = tf.reshape(
                self.observations_after,
                [batch_size, self.look_back, self.state_dimension_per_agent])
            # Extract the pose of the agent after the action was performed
            self.pose_state_after = tf.reshape(
                self.state_after[:, :, :self.size_pose_state],
                [batch_size, self.size_pose_state * self.look_back])
            # Extract the occupancy grid of the agent after the action was performed
            extracted_observation_map_after = tf.transpose(
                self.state_after[:, :, self.size_pose_state:], perm=[0, 2, 1])
            self.observation_map_after = tf.reshape(
                extracted_observation_map_after,
                [batch_size, self.observation_map_dimension, self.observation_map_dimension,
                self.look_back])

            # Create neural network architecture for Soft Actor Critic algorithm
            with tf.variable_scope('main'):
                self.mu, self.pi, logp_pi, q1, q2, q1_pi, q2_pi = soft_actor_critic_architecture(
                    self.pose_state_before, self.observation_map_before, self.action, action_scale)
            with tf.variable_scope('target'):
                _, _, logp_pi_target,\
                    _, _, q1_pi_target, q2_pi_target = soft_actor_critic_architecture(
                    self.pose_state_after, self.observation_map_after, self.action, action_scale)

            target_entropy = tf.cast(-self.action_dimension_per_agent, tf.float32)

            # Extract the exploration parameter alpha
            log_alpha = tf.get_variable('log_alpha', dtype=tf.float32, initializer=0.0)
            alpha = tf.exp(log_alpha)

            # Min Double-Q:
            min_q_pi = tf.minimum(q1_pi, q2_pi)
            min_q_pi_target = tf.minimum(q1_pi_target, q2_pi_target)

            # Targets for Q value regression
            q_backup = self.reward + gamma * (
                1 - self.death) * tf.stop_gradient(min_q_pi_target - alpha * logp_pi_target)

            # Soft actor-critic losses
            q1_loss = 0.5 * tf.reduce_mean((q_backup - q1)**2)
            q2_loss = 0.5 * tf.reduce_mean((q_backup - q2)**2)

            value_loss = q1_loss + q2_loss

            pi_loss = tf.reduce_mean(
                alpha * logp_pi - min_q_pi)    # We can subtract with min_q_pi here as well

            # alpha loss for temperature parameter
            alpha_backup = tf.stop_gradient(logp_pi + target_entropy)
            alpha_loss = -tf.reduce_mean(log_alpha * alpha_backup)

            # Plot values on Tensorboard
            tf.summary.scalar("pi_Loss", pi_loss)
            tf.summary.scalar("q1_Loss", q1_loss)
            tf.summary.scalar("q2_Loss", q2_loss)
            tf.summary.scalar("alpha_Loss", alpha_loss)

            # Policy training operation
            pi_optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate, epsilon=1e-04)
            train_policy_op = pi_optimizer.minimize(pi_loss, var_list=get_trainable_vars('main/pi'))

            # Value loss training operation
            value_optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate, epsilon=1e-04)
            with tf.control_dependencies([train_policy_op]):
                train_value_function_op = value_optimizer.minimize(
                    value_loss, var_list=get_trainable_vars('main/q'))

            # Alpha loss training operation
            alpha_optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate, epsilon=1e-04)
            with tf.control_dependencies([train_value_function_op]):
                train_alpha_op = alpha_optimizer.minimize(
                    alpha_loss, var_list=get_trainable_vars('log_alpha'))

            # Polyak averaging for target variables
            with tf.control_dependencies([train_value_function_op]):
                target_update = tf.group([
                    tf.assign(v_targ, polyak * v_targ + (1 - polyak) * v_main)
                    for v_main, v_targ in zip(
                        get_trainable_vars('main'), get_trainable_vars('target'))
                ])

            # Merge all summaries together and set log writer for Tensorboard
            merged_summary = tf.summary.merge_all()
            self.train_writer = tf.summary.FileWriter(log_dir, self.sess.graph)

            # All ops to call during one training step
            self.step_ops = [
                pi_loss, q1_loss, q2_loss, q1, q2, logp_pi, target_entropy, alpha_loss, alpha,
                train_policy_op, train_value_function_op, train_alpha_op, target_update,
                merged_summary
            ]
            # Initializing targets to match main variables
            self.target_initialization = tf.group([
                tf.assign(v_targ, v_main)
                for v_main, v_targ in zip(get_trainable_vars('main'), get_trainable_vars('target'))
            ])
            # Setup checkpoint saver
            self.saver = tf.train.Saver(max_to_keep=None)
            # Initialize the network
            self.sess.run(tf.global_variables_initializer())
            self.sess.run(self.target_initialization)

    def predict_action(self, observations, deterministic=False):
        """Given state as input, output the optimal action for a given state"""
        act_op = self.mu if deterministic else self.pi
        # Run inference on neural network
        return self.sess.run(act_op, feed_dict={self.observations_before: observations})[0]

    def train(self, observations_before, observations_after, action, reward, death):
        """Function to train neural network given training data"""
        self.training_step_count += 1
        feed_dict = {
            self.observations_before: observations_before,
            self.observations_after: observations_after,
            self.action: action,
            self.reward: reward,
            self.death: death,
        }
        _, _, _, _, _, _, _, _, _, _, _, _, _, summary = self.sess.run(
            self.step_ops, feed_dict=feed_dict)
        # Store summary to Tensorboard logs
        self.train_writer.add_summary(summary, self.training_step_count)

    def restore(self, checkpoint_path):
        """Restore session and graph given checkpoint_path"""
        with self.graph.as_default():
            self.sess = tf.Session(graph=self.graph)
            tf.train.Saver().restore(self.sess, checkpoint_path)

    def save_policy(self, checkpoint_path, id):
        """Save policy checkpoints"""
        self.saver.save(self.sess, checkpoint_path + str(id) + ".ckpt")
