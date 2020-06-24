'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import numpy as np
import time
from engine.pyalice import Message


class OffPolicyTrainer:
    """
    Trains the soft-actor-critic algorithm : https://arxiv.org/abs/1812.05905
    The codelet expects the user has already set the policy and sample accumulator
    bridge for the codelet in the main application. It takes in the following
    config parameters:
        app : Isaac application that will be pinged to publish and receiving messages
        policy : reinforcement learning neural network
        bridge : bridge to the sample accumulator (PybindSampleAccumulator)
        buffer_threshold : size of the sample accumulator buffer needed to start training
        size_pose_state : length of the pose element for each timestep
        observation_map_dimension : dimension of the occupancy grid square (length of the side)
        max_steps_per_epoch : maximum number of training steps for each epoch
        max_episode_length : maximum length an episode can last if the agent does not die
        use_pretrained_model : true, if training needs to be resumed from a stored checkpoint
        restore_path : absolute path to the stored checkpoint if pretrained model is used
        num_agents : total number of agents across all sims
        look_back : number of past timesteps batched together to form state
        mode : "train" for running training or "play" for running inference
        zero_action : numpy array containing the action to stop the robot
        batch_size : size of the batch for training neural network
        state_dim_per_agent : size of the state space
        action_dim_per_agent : size of the action space
        tuple_dimension : row size of the samples stored in sample accumulator
        aux_end_of_episode_flag : index of flag in aux to signify agent has reached maximum age
        checkpoint_directory : directory in which tensorflow checkpoints are saved
        log_directory : directory in which logs regarding the training are stored
    """

    def __init__(self, app, policy, bridge, buffer_threshold, size_pose_state,
                 observation_map_dimension, max_steps_per_epoch, max_episode_length,
                 aux_end_of_episode_flag, action_scale, experience_buffer_size, look_back,
                 checkpoint_directory, num_agents, mode, state_dim_per_agent,
                 action_dim_per_agent, tuple_dimension, batch_size, zero_action,
                 log_directory, restore_path, use_pretrained_model):
        self.app = app
        self.buffer_threshold = buffer_threshold
        self.policy = policy
        self.bridge = bridge
        self.size_pose_state = size_pose_state
        self.observation_map_dimension = observation_map_dimension
        self.max_steps_per_epoch = max_steps_per_epoch
        self.max_episode_length = max_episode_length
        self.experience_buffer_size = experience_buffer_size
        self.look_back = look_back
        self.action_scale = action_scale
        self.zero_action = zero_action
        self.aux_end_of_episode_flag = aux_end_of_episode_flag
        self.checkpoint_directory = checkpoint_directory
        self.num_agents = num_agents
        self.state_dim_per_agent = state_dim_per_agent
        self.action_dim_per_agent = action_dim_per_agent
        self.tuple_dimension = tuple_dimension
        self.batch_size = batch_size
        self.mode = mode
        self.log_directory = log_directory
        self.restore_path = restore_path
        self.last_sample_accumulator_size = self.buffer_threshold
        self.use_pretrained_model = use_pretrained_model

        # Restore the pretrained model for training or inference
        if self.use_pretrained_model:
            self.policy.restore(self.restore_path)
        # Store sum of rewards for all the episodes in the training
        self.sum_episode_reward = 0.0
        # Store the sum of ages of all the agents throughout the training
        self.sum_agent_life = 0
        # Number of episodes completed
        self.num_episodes = 0
        # Number of checkpoints stored
        self.checkpoint_iterations = 0

    def get_action(self, observations, step_count):
        """
        Given the current state of the robot, find the next action to perform
        """
        # If only running inference
        if self.mode != "train":
            return self.policy.predict_action(observations, deterministic=True)

        # If the sample accumulator is not full, pick a random action to perform
        if self.bridge.get_sample_count() < self.buffer_threshold:
            return np.random.uniform(-1.0 * self.action_scale,
                                     self.action_scale,
                                     self.action_dim_per_agent)
        else:
            # Send a stop signal if the training episode is about to begin
            if step_count % self.max_episode_length == 0:
                return self.zero_action
            else:
                return self.policy.predict_action(observations)

    def train(self):
        """
        Runs a single epoch of training by extracting data from sample accumulator
        and running policy backpropogation.
        """
        # If the sample accumulator does not have sufficient data
        sample_accumulator_size = self.bridge.get_sample_count()
        if sample_accumulator_size < self.buffer_threshold:
            return

        if sample_accumulator_size == self.experience_buffer_size:
            epochs = self.max_steps_per_epoch
        else:
            # Find the number of new samples in the sample accumulator
            # Generally the model needs to update once for every new piece of data
            epochs = sample_accumulator_size - self.last_sample_accumulator_size

        self.last_sample_accumulator_size = sample_accumulator_size

        # Run through all epochs
        for _ in range(np.minimum(epochs, self.max_steps_per_epoch)):
            # Query the sample accumulator for samples to train
            sample_accumulator_batch = np.squeeze(np.array(
                self.bridge.peak_random_samples(self.batch_size)))
            # Extract the batch of past observations
            observations_before = np.array([
                sample_accumulator_batch[:, 1][i]
                for i in range(sample_accumulator_batch.shape[0])
            ])

            # Extract the actions that were performed for the previous state
            actions = np.array([
                sample_accumulator_batch[:, 2][i]
                for i in range(sample_accumulator_batch.shape[0])
            ])

            # Extract the batch of observations after performing an action
            observations_after = np.array([
                sample_accumulator_batch[:, 0][i]
                for i in range(sample_accumulator_batch.shape[0])
            ])

            # Extract batch of rewards received
            rewards = np.squeeze([
                sample_accumulator_batch[:, 3][i]
                for i in range(sample_accumulator_batch.shape[0])
            ])


            # Extract the dead or alive flags
            dead = np.squeeze([
                sample_accumulator_batch[:, 4][i]
                for i in range(sample_accumulator_batch.shape[0])
            ])

            # Send the batch for training
            self.policy.train(observations_before, observations_after, actions,
                              rewards, dead)
        # Create checkpoints and update logs after every checkpoint_frequency steps
        self.checkpoint_iterations += 1
        # Store the self.policy as tensorflow checkpoint
        self.policy.save_policy(self.checkpoint_directory + "/agent", self.checkpoint_iterations)
        # Log data
        if self.num_episodes > 0:
            # Store the current average episode per reward
            with open(self.log_directory + "/rewards.txt", "a") as file:
                file.write("{}\n".format(self.sum_episode_reward / self.num_episodes))
            # Store the current average life of the agent
            with open(self.log_directory + "/age.txt", "a") as file:
                file.write("{}\n".format(self.sum_agent_life / self.num_episodes))

    def main(self):
        """
        Runs the training loop while performing inference from the reinforcement learning policy
        """
        # Store intra episodic rewards for each agent
        agent_reward = np.zeros(self.num_agents)
        # Store the age (number of timesteps) for each agent
        agent_life = np.zeros(self.num_agents, dtype=int)

        step_count = 0

        # Main training loop
        while True:
            # Receive data for inference from the batching codelet
            state_tensor = self.app.receive("temporal_batching", "TemporalBatching", "temporal_tensor")
            if state_tensor == None:
                # If no new messages are received
                time.sleep(0.05)
                continue

            # Extract data from the received state message
            state_buffer = np.frombuffer(state_tensor.buffers[0], dtype=np.float32)
            state_buffer = np.reshape(state_buffer,
                (self.num_agents, self.look_back*self.state_dim_per_agent))

            # Extract the message from the reward tensor
            reward_tensor = self.app.receive("state_machine_gym_flow",
                                             "StateMachineGymFlow", "transition_reward")
            if reward_tensor == None:
                raise ValueError('Reward tensor not received for the current transition')
            reward_buffer = np.frombuffer(reward_tensor.buffers[0], dtype=np.float32)
            reward_buffer = np.reshape(reward_buffer, (self.num_agents))

            # Extract the dead tensor from Gym
            dead_tensor = self.app.receive("state_machine_gym_flow",
                                           "StateMachineGymFlow", "transition_dead")
            if dead_tensor == None:
                raise ValueError('Dead tensor not received for the current transition')
            dead_buffer = np.frombuffer(dead_tensor.buffers[0], dtype=np.float32)
            dead_buffer = np.reshape(dead_buffer, (self.num_agents))

            # Extract the age tensor from Gym
            aux_tensor = self.app.receive("state_machine_gym_flow",
                                          "StateMachineGymFlow", "transition_aux")
            if aux_tensor == None:
                raise ValueError('Aux tensor not received for the current transition')
            aux_buffer = np.frombuffer(aux_tensor.buffers[0], dtype=np.float32)
            aux_buffer = np.reshape(aux_buffer, (self.num_agents, -1))
            age_buffer = aux_buffer[:, self.aux_end_of_episode_flag]

            # Increment step counter
            step_count = step_count + 1
            # Create the output array to store actions to perform
            action_array = np.zeros((self.num_agents, self.action_dim_per_agent), dtype=np.float32)

            # For each agent, pass the state to the neural network and store the inferred output
            for i in range(self.num_agents):
                # Extract tuple for each agent
                agent_tuple = state_buffer[i]
                # Extract the latest state observed by the agent
                observations = agent_tuple.reshape(1, -1)
                # Extract the reward received by the agent
                reward = reward_buffer[i]
                # Extract the flag indicating if the agent is dead or alive
                dead = dead_buffer[i]
                # Extract flag indicating if the robot has reached its maximum age
                age = age_buffer[i]
                # Get action to be performed from the policy
                action_array[i] = self.get_action(observations, step_count)
                # Append current reward with the episodic reward
                agent_reward[i] += reward
                # Increment age of agent by 1
                agent_life[i] += 1
                # If agent is dead
                # Note : A check on dead and age is needed
                # Age flag is reset to avoid undue penalty while training
                if (dead == 1.0 or age == 1.0):
                    # Since episode has completed, append episode reward
                    self.sum_episode_reward += agent_reward[i]
                    # Append final age of agent before dying to the sum
                    self.sum_agent_life += agent_life[i]
                    self.num_episodes += 1
                    # Reset the counters
                    agent_reward[i] = 0.0
                    agent_life[i] = 0
            # Send the actions inferred from the self.policy
            send_msg = Message.create_message_builder("TensorProto")
            send_msg.proto.elementType = "float32"
            send_msg.proto.sizes = [self.num_agents, self.action_dim_per_agent]
            send_msg.proto.dataBufferIndex = 0
            send_msg.buffers = [action_array]
            self.app.publish("state_machine_gym_flow", "StateMachineGymFlow",
                             "action", send_msg)
            # Start training after self.max_episode_length is reached
            if step_count % self.max_episode_length == 0 and self.mode == "train":
                self.train()
                step_count = 0
