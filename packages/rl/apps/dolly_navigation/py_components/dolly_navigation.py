'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

from engine.pyalice import *
from pathlib import Path

import argparse
import json
import numpy
import os
import packages.ml


def load_config(training_config_path):
    """Stores JSON config into a python dictionary

    Args:
        training_config_path: Path to the json config file

    Returns:
        Dictionary containing the config

    """
    config = {}
    config_path = os.fspath(Path(training_config_path).resolve())
    with open(config_path) as config_file:
        config = json.load(config_file)
    # Compute the final state dimension
    config['state_dimension'] = config['size_pose_state'] + (
        config['observation_map_dimension'] * config['observation_map_dimension'])
    # Compute total number of agents across all simulation instances
    config['num_agents'] = int(config['num_agents_per_sim'] * config['sim_instances'])
    return config


def parse_key_to_config(component, config, selected_keys):
    """Stores the config into the component if the name of the key
    in the config matches the name of the parameter in the codelet

    Args:
        component: codelet whose config needs to be set
        config: configuration dictionary
        selected_keys: dictionary keys that need to be copied into the
                       component config

    """
    for key in selected_keys:
        component.config[key] = config[key]


def get_sim_and_agent_index(agent_id, config):
    """Given the index of the agent, find which simulator and
    which position the agent belongs to (inside the simulator)

    Args:
        agent_id: Global index of the agent
        config: configuration directory
    Returns:
        sim_index : which simulator the agent belongs to
        agent_index : the index of the agent inside the simulator

    """
    # Given the position of the agent in the array
    # return which simulation instance and which position
    # inside the simulation the agent belongs to
    sim_index = agent_id // config['num_agents_per_sim']
    agent_index = agent_id % config['num_agents_per_sim']
    return sim_index, agent_index


def create_dolly_docking_pipeline(app, config):
    """Creates the dolly docking application

    Args:
        app: Isaac SDK application
        config: configuration dictionary

    """
    num_agents = config['num_agents']
    # Create connections to each simulation instance
    # When multiple simulations are connected, the assumption
    # is that simulators have ports starting from 55000 and increasing
    # in steps of 10
    PORT_INCREMENT = 10
    PORT_START_NUM = 55000
    tcp_sub_array = []
    tcp_pub_array = []
    for i in range(config['sim_instances']):
        simulation = app.add('simulation_{}'.format(i + 1))
        tcp_sub = simulation.add(app.registry.isaac.alice.TcpSubscriber)
        tcp_pub = simulation.add(app.registry.isaac.alice.TcpPublisher)
        simulation.add(app.registry.isaac.alice.TimeSynchronizer)
        tcp_sub.config['port'] = PORT_START_NUM
        tcp_sub.config['host'] = 'localhost'
        tcp_pub.config['port'] = PORT_START_NUM + 1
        tcp_sub_array.append(tcp_sub)
        tcp_pub_array.append(tcp_pub)
        PORT_START_NUM += PORT_INCREMENT

    # Creates the odometry codelets to receive state information from sim
    # Each agent has their own codelet
    # Note: Odometry codelets need to be created before Coach in order
    # to reference them properly
    odometry_array = []
    for i in range(num_agents):
        odometry = app.add('odometry_{}'.format(i + 1))
        codelet = odometry.add(app.registry.isaac.navigation.DifferentialBaseOdometry)
        codelet.config['tick_period'] = '100Hz'
        codelet.config['odometry_frame'] = 'odom_{}'.format(i + 1)
        codelet.config['robot_frame'] = 'robot_{}'.format(i + 1)
        sim_index, agent_index = get_sim_and_agent_index(i, config)
        app.connect(tcp_sub_array[sim_index],
                    'base_state{}'.format(agent_index + 1), codelet, 'state')
        odometry_array.append(codelet)

    # Creates the DifferentialBaseControl codelets to send messages to sim
    # Each agent has their own codelet
    # Note: DifferentialBaseControl codelets need to be created before Coach
    # in order to reference them properly
    pid_array = []
    for i in range(num_agents):
        pid = app.add('pid_{}'.format(i + 1))
        codelet = pid.add(app.registry.isaac.planner.DifferentialBaseControl)
        codelet.config['tick_period'] = '100Hz'
        codelet.config['robot_frame'] = 'robot_{}'.format(i + 1)
        codelet.config['static_frame'] = 'odom_{}'.format(i + 1)
        sim_index, agent_index = get_sim_and_agent_index(i, config)
        pid_array.append(codelet)
        app.connect(codelet, 'cmd', tcp_pub_array[sim_index],
                    'base_command{}'.format(agent_index + 1))

    # Creates the state machine for reinforcement learning training and inference
    state_machine_gym_flow = app.add('state_machine_gym_flow')
    state_machine_gym_flow.add(app.registry.isaac.alice.Random)

    for i in range(num_agents):
        # Create Birth components that spawns agents within the simulation
        # The reset encoder also forwards the teleport messages to their individual sim targets
        # Create one Birth component for each agent in simulation
        reset_codelet = state_machine_gym_flow.add(
            app.registry.isaac.rl.DollyDockingBirth, name='birth_{}'.format(i + 1))
        parse_key_to_config(reset_codelet, config, [
            'agents_per_row', 'start_coordinate', 'target_separation',
            'obstacle_separation', 'agent_spawn_randomization',
            'target_spawn_randomization', 'obstacle_spawn_randomization',
            'obstacle_scale_randomization', 'dividing_space'
        ])
        sim_index, agent_index = get_sim_and_agent_index(i, config)
        app.connect(reset_codelet, 'sim{}'.format(i + 1), tcp_pub_array[sim_index],
                    'teleport{}'.format(agent_index + 1))

        # Create Reward components to compute rewards for the action of each agent
        # Create one Reward component for each agent in simulation
        reward = state_machine_gym_flow.add(
            app.registry.isaac.rl.DollyDockingReward, name='reward_{}'.format(i + 1))
        parse_key_to_config(reward, config, [
            'collision_penalty', 'success_reward', 'reward_clip_range', 'tolerance',
            'bias', 'ideal_docking_pose'
        ])

        # Create Death components to decide if an agent is in invalid state
        # Create one Death component for each agent in simulation
        death = state_machine_gym_flow.add(
            app.registry.isaac.rl.DollyDockingDeath, name='death_{}'.format(i + 1))
        parse_key_to_config(death, config,
                            ['x_allowance', 'y_allowance', 'angle_allowance'])
        death.config['maximum_age'] = config['max_episode_length']

        # Create Noiser components to add noise to each agent and convert pose of
        # the dolly from odom to robot coordinates
        # Create one Refiner component for each agent in simulation
        refiner = state_machine_gym_flow.add(
            app.registry.isaac.rl.DollyDockingStateNoiser,
            name='refiner_{}'.format(i + 1))
        parse_key_to_config(refiner, config, ["target_pose_noise"])

    # Create the gym component
    gym = state_machine_gym_flow.add(app.registry.isaac.rl.StateMachineGymFlow)
    parse_key_to_config(gym, config, [
        'tick_period', 'num_agents', 'state_dimension', 'action_dimension',
        'aux_dimension', 'delay_sending', 'reaction_time'
    ])
    gym.config['episode_end_flag_index'] = config['aux_end_of_episode_flag']

    # Adds the state aggregator to aggregate state data from individual agents
    state_tensor_aggregator = app.add('state_tensor_aggregator')
    state_aggregator_component = state_tensor_aggregator.add(
        app.registry.isaac.rl.TensorAggregator)
    state_aggregator_component.config['tick_period'] = config['tick_period']
    app.connect(state_aggregator_component, 'aggregate_tensor', gym, 'state')

    # Adds the aux state aggregator to aggregate aux data from individual agents
    aux_tensor_aggregator = app.add('aux_tensor_aggregator')
    aux_aggregator_component = aux_tensor_aggregator.add(
        app.registry.isaac.rl.TensorAggregator)
    aux_aggregator_component.config['tick_period'] = config['tick_period']
    app.connect(aux_aggregator_component, 'aggregate_tensor', gym, 'aux')

    # Adds the deaggregator to aggregate action data from individual agents
    tensor_deaggregator = app.add('tensor_deaggregator')
    state_deaggregator_component = tensor_deaggregator.add(
        app.registry.isaac.rl.TensorDeaggregator)
    app.connect(gym, 'command', state_deaggregator_component, 'aggregate_tensor')

    # Add temporal batching codelet for store batches of historical data
    temporal_batching = app.add('temporal_batching')
    batching_codelet = temporal_batching.add(app.registry.isaac.rl.TemporalBatching)
    parse_key_to_config(batching_codelet, config, ['num_agents', 'look_back'])
    batching_codelet.config['reset_interval'] = config['max_episode_length']
    batching_codelet.config['episode_end_flag_index'] = config['aux_end_of_episode_flag']

    # Receive tuples from Gym in TemporalBatching
    app.connect(gym, 'transition_state', batching_codelet, 'step')
    app.connect(gym, 'transition_action', batching_codelet, 'action')
    app.connect(gym, 'transition_reward', batching_codelet, 'reward')
    app.connect(gym, 'transition_dead', batching_codelet, 'dead')
    app.connect(gym, 'transition_aux', batching_codelet, 'aux')
    sample_accumulator = app.add('sample_accumulator')
    sample_accumulator_component = sample_accumulator.add(
        app.registry.isaac.ml.SampleAccumulator)
    sample_accumulator.add(app.registry.isaac.alice.Random)
    # The sample accumalator is not needed during inference
    if config['mode'] == 'train':
        # Connect the sample accumulator component to store Occupancy grid samples
        app.connect(batching_codelet, 'state_samples',
                    sample_accumulator_component, 'state_samples')
        app.connect(batching_codelet, 'last_state_samples',
                    sample_accumulator_component, 'last_state_samples')
        app.connect(batching_codelet, 'reward_samples',
                    sample_accumulator_component, 'reward_samples')
        app.connect(batching_codelet, 'action_samples',
                    sample_accumulator_component, 'action_samples')
        app.connect(batching_codelet, 'dead_samples',
                    sample_accumulator_component, 'dead_samples')
        sample_accumulator_component.config['sample_buffer_size'] = config[
            'experience_buffer_size']
        sample_accumulator_component.config['channel_names'] = ['state_samples',
        'last_state_samples', 'action_samples', 'reward_samples', 'dead_samples']
    else:
        batching_codelet.config['reset_interval'] = -1

    # Adds the state decoder codelets to aggregate odometry and occupancy grid information
    state_decoder_array = []
    for i in range(num_agents):
        state_decoder = app.add('state_decoder_{}'.format(i + 1))
        codelet = state_decoder.add(app.registry.isaac.rl.DollyDockingStateDecoder)
        codelet.config['tick_period'] = config['tick_period']
        app.connect(codelet, 'agent_state', state_aggregator_component, 'agent{}'.format(i + 1))
        app.connect(odometry_array[i], 'odometry', codelet, 'base_state')
        state_decoder_array.append(codelet)

    # Adds codelets to compute the range scan from the lidar information
    range_scan_array = []
    for i in range(num_agents):
        range_scan = app.add('range_scan_{}'.format(i + 1))
        codelet = range_scan.add(app.registry.isaac.perception.RangeScanFlattening)
        sim_index, agent_index = get_sim_and_agent_index(i, config)
        app.connect(tcp_sub_array[sim_index],
                    'rangescan{}'.format(agent_index + 1), codelet, 'scan')
        range_scan_array.append(codelet)

    # Adds the velocity integrator array that creates a valid plan for the controller
    velocity_integrator_array = []
    for i in range(num_agents):
        velocity_integrator = app.add('velocity_integrator_{}'.format(i + 1))
        codelet = velocity_integrator.add(
            app.registry.isaac.planner.DifferentialBaseVelocityIntegrator)
        codelet.config['delta_time'] = 0.001
        velocity_integrator_array.append(codelet)

    # Creates the composite to differential base trajectory codelets
    composite_to_diff_trajectory_array = []
    for i in range(num_agents):
        composite_to_diff_trajectory = app.add('composite_to_diff_trajectory_{}'.format(i + 1))
        codelet = composite_to_diff_trajectory.add(
            app.registry.isaac.utils.CompositeToDifferentialTrajectoryConverter)
        codelet.config['frame'] = 'robot_{}'.format(i + 1)
        app.connect(velocity_integrator_array[i], 'plan', codelet, 'input_plan')
        app.connect(codelet, 'output_plan', pid_array[i], 'plan')
        composite_to_diff_trajectory_array.append(codelet)

    # Creates the aux decoder codelets that aggregates collision information with
    # ground truth poses
    for i in range(num_agents):
        aux_decoder = app.add('aux_decoder_{}'.format(i + 1))
        codelet = aux_decoder.add(app.registry.isaac.rl.DollyDockingAuxDecoder)
        app.connect(codelet, 'agent_aux', aux_aggregator_component, 'agent{}'.format(i + 1))
        sim_index, agent_index = get_sim_and_agent_index(i, config)
        app.connect(tcp_sub_array[sim_index],
                    'bodies{}'.format(agent_index + 1), codelet, 'scene_pose')
        app.connect(tcp_sub_array[sim_index], 'collision{}'.format(agent_index + 1), codelet,
                    'body_collision')

    # Adds the VelocityProfileEncoder codelets that converts tensors received from the
    # neural network into Velocity Profile
    for i in range(num_agents):
        velocity_profile_encoder = app.add('velocity_profile_encoder_{}'.format(i + 1))
        codelet = velocity_profile_encoder.add(
            app.registry.isaac.rl.TensorToCompositeVelocityProfile)
        codelet.config['input_linear_velocity_range'] = [
            -1.0 * config['action_scale'], config['action_scale']
        ]
        codelet.config['input_angular_velocity_range'] = [
            -1.0 * config['action_scale'], config['action_scale']
        ]
        parse_key_to_config(codelet, config, [
            'timestamp_profile', 'output_linear_velocity_range', 'output_angular_velocity_range'
        ])
        app.connect(state_deaggregator_component, 'agent{}'.format(i + 1),
                    codelet, 'policy')
        app.connect(codelet, 'velocity_profile', velocity_integrator_array[i], 'target_velocities')

    # Creates the codelets that converts Flatscan to Occupancy Grid
    for i in range(num_agents):
        ogm = app.add('ogm_{}'.format(i + 1))
        codelet = ogm.add(app.registry.isaac.navigation.RangeScanToObservationMap)
        # OGM is fixed at [128,128] for now
        codelet.config['dimensions'] = [128, 128]
        parse_key_to_config(codelet, config, ['cell_size', 'wall_thickness'])
        codelet.config['sensor_frame'] = "lidar_{}".format(i + 1)
        codelet.config['sensor_lattice_frame'] = "lidar_lattice_{}".format(i + 1)
        app.connect(range_scan_array[i], 'flatscan', codelet, 'flatscan')
        app.connect(codelet, 'observation_map', state_decoder_array[i], 'local_map')


def py_reinforcement_learning_app(config):
    from packages.rl.py_components.soft_actor_critic import SoftActorCriticPolicy
    from packages.rl.py_components.off_policy_trainer import OffPolicyTrainer
    # Create application
    app = Application(
        name = 'dolly_navigation',
        modules = [
            'ml', 'navigation', 'planner', 'perception',
            '//packages/rl/components:rl',
            '//packages/rl/apps/dolly_navigation/components:dolly_navigation_task',
            'utils'
        ])
    # Add the codelets to the reinforcement learning pipeline along with relevant
    # connections and config
    create_dolly_docking_pipeline(app, config)
    # Create the reinforcement learning policy
    policy = SoftActorCriticPolicy(
        size_pose_state = config['size_pose_state'],
        observation_map_dimension = config['observation_map_dimension'],
        look_back = config['look_back'],
        size_action_space = config['action_dimension'],
        action_scale = config['action_scale'],
        learning_rate = config['learning_rate'],
        polyak = config['polyak'],
        gamma = config['gamma'],
        log_dir = config['tensorboard_log_directory'])
    # If mode = inference, set use_pretrained_model flag to true as well
    if config['mode'] != "train":
        config['use_pretrained_model'] = True
    # Set the sample accumulator bridge in the training codelet
    accumulator_host = app.nodes['sample_accumulator']
    assert accumulator_host is not None
    accumulator_bridge = packages.ml.SampleAccumulator(accumulator_host._node)
    # Start the app
    app.start()
    # Create an instance of the trainer
    trainer = OffPolicyTrainer(
        app = app,
        policy = policy,
        bridge = accumulator_bridge,
        buffer_threshold = config['buffer_threshold'],
        size_pose_state = config['size_pose_state'],
        observation_map_dimension = config['observation_map_dimension'],
        action_scale = config['action_scale'],
        experience_buffer_size = config['experience_buffer_size'],
        max_steps_per_epoch = config['max_steps_per_epoch'],
        max_episode_length = config['max_episode_length'],
        look_back = config['look_back'],
        aux_end_of_episode_flag = config['aux_end_of_episode_flag'],
        checkpoint_directory = config['checkpoint_directory'],
        num_agents = config['num_agents'],
        state_dim_per_agent = config['state_dimension'],
        action_dim_per_agent = config['action_dimension'],
        tuple_dimension = (config['state_dimension'] * 2) +
        config['action_dimension'] + 1 + 1 + config['aux_dimension'],
        zero_action = numpy.array([
            -1.0 * config['action_scale'], 0.0, -1.0 * config['action_scale'],
            0.0, -1.0 * config['action_scale'], 0.0
        ]),
        mode = config['mode'],
        batch_size = config['batch_size'],
        log_directory = config['tensorboard_log_directory'],
        restore_path = config['restore_path'],
        use_pretrained_model = config['use_pretrained_model'])
    try:
        # Start the training loop
        trainer.main()
    except KeyboardInterrupt:
        print('Exiting due to keyboard interrupt')
        app.stop()
    app.stop()

def frozen_model_inference_app(config):
    # Create application
    app = Application(
        name = 'dolly_navigation',
        modules = [
            'ml', '//packages/ml:tensorflow', 'navigation', 'planner', 'perception',
            '//packages/rl/components:rl',
            '//packages/rl/apps/dolly_navigation/components:dolly_navigation_task',
            'utils'
        ])
    # Add the codelets to the reinforcement learning pipeline along with relevant
    # connections and config
    create_dolly_docking_pipeline(app, config)
    # Adds TensorflowInference codelet for CPP inference
    frozen_model_node = app.add('frozen_model')
    frozen_model_codelet = frozen_model_node.add(app.registry.isaac.ml.TensorflowInference)
    restore_path = config['restore_path']
    # Assume the path to config is same as path to frozen model
    config_path = restore_path[:restore_path.rfind('/')+1] + 'allow_growth.cfg'
    frozen_model_codelet.config['model_file_path'] = restore_path
    frozen_model_codelet.config['config_file_path'] = config_path
    # Set expected tensor information
    frozen_model_codelet.config['input_tensor_info'] = [{
        "ops_name": "observations_before",
        "index": 0,
        "dims": [config['num_agents'], config['look_back'] * config['state_dimension']],
    }]
    frozen_model_codelet.config['output_tensor_info'] = [{
        "ops_name": "main/mul",
        "channel": "output",
        "index": 0,
        "dims": [config['num_agents'], config['action_dimension']],
    }]
    # Send outputs back to Gym after taking in output of the TemporalBatcher
    app.connect(app.nodes['temporal_batching']['TemporalBatching'], 'temporal_tensor',
                app.nodes['frozen_model']['TensorflowInference'], 'observations_before')
    app.connect(app.nodes['frozen_model']['TensorflowInference'], 'output',
                app.nodes['state_machine_gym_flow']['StateMachineGymFlow'], 'action')

    app.run()

def main(config):
    if config['mode'] == "inference_pb":
        # If frozen model needs to be used for inference
        frozen_model_inference_app(config)
    else:
        # For python training or inference with model checkpoint
        py_reinforcement_learning_app(config)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Parameters for Reinforcement Learning Pipeline')
    parser.add_argument(
        '--config',
        dest='config',
        default='packages/rl/apps/dolly_navigation/py_components/trainer.config.json',
        help='The path to the config file for running training or inference')
    args = parser.parse_args()
    config = load_config(args.config)
    main(config)