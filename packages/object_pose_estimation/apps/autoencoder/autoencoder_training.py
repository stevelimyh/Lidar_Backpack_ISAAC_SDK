'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
"""
Autoencoder training script.

Trains a neural network which takes in RGB images of different orientations of
an object and generates an embedding which is independent of backgrounds and scale.
The network architecture used is based on AugmentedAutoencoder.

This script is setup to stream images from Isaac SDK for training.
"""

from engine.pyalice import *
from pathlib import Path

import gc
import json
import numpy as np
import os
import packages.ml
import packages.object_pose_estimation.apps.autoencoder
import shutil
import tensorflow as tf
import time

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"    # so the IDs match nvidia-smi
os.environ["CUDA_VISIBLE_DEVICES"] = "0"    # Limit the GPU usage to gpu #0

from tensorflow.python.client import timeline
from tensorflow.python.tools import freeze_graph
from tensorflow.python.tools import optimize_for_inference_lib

from autoencoder_utils import  build_ae, build_encoder, build_decoder, build_train_op

# Command line flags:
flags = tf.app.flags
FLAGS = flags.FLAGS
flags.DEFINE_string('training_config_path',
                    'packages/object_pose_estimation/apps/autoencoder/training_config.json',
                    'Relative path to the config file containing training configurations')

# Op names.
COLOR_IMAGE = 'rgb_image'
RECONSTRUCT_IMAGE = 'reconst_img'

def get_generator(bridge, config):
    """Create a training sample generator.

    Args:
      bridge: the isaac sample accumulator node which we will acquire samples from

    Returns:
      A generator function which yields a single training example.
    """
    def _generator():
        # Indefinitely yield samples. Requires the training scene to be running on navsim.
        while True:
            # Try to acquire a sample.
            # If none are available, then wait for a bit so we do not spam the app.
            samples = bridge.acquire_samples(config['batch_size'])
            if not samples:
                time.sleep(1)
                continue
            if len(samples[0][1]) == 0:
                continue

            for i in range(len(samples)):
                # Encoder image
                encoder_img = np.rollaxis(samples[i][0], 1, 4)
                encoder_img = np.squeeze(encoder_img)
                # Continue if it is empty tensor
                if (encoder_img.shape[0] == 0):
                    continue

                # Decoder image
                decoder_img = np.rollaxis(samples[i][1], 1, 4)
                decoder_img = np.squeeze(decoder_img)

                yield {COLOR_IMAGE: encoder_img, RECONSTRUCT_IMAGE: decoder_img}
    return _generator

def get_dataset(bridge, config):
    """Create a tf.data dataset which yields batches of samples for training.

    Args:
      bridge: the isaac sample accumulator node which we will acquire samples from

    Returns:
      A tf.data dataset which yields batches of training examples.
    """
    dataset = tf.data.Dataset.from_generator(
        get_generator(bridge, config), {
            COLOR_IMAGE: tf.float32,
            RECONSTRUCT_IMAGE: tf.float32,
        }, {
            COLOR_IMAGE: tf.TensorShape([int(128), int(128), int(3)]),
            RECONSTRUCT_IMAGE: tf.TensorShape([int(128), int(128), int(3)]),
        })
    dataset = dataset.batch(batch_size=config['batch_size'])
    dataset = dataset.prefetch(buffer_size=tf.data.experimental.AUTOTUNE)
    return dataset

def make_dir(path):
    """Clears a directory and creates it on disk.

    Args:
      path: The path on disk where the directory should be located.
    """

    # Catch an exception when trying to delete the path because it may not yet exist.
    try:
        shutil.rmtree(path)
    except:
        pass

    # Create the directory. If it already exists, the error will get excepted
    try:
        os.makedirs(path)
    except FileExistsError:
        # If the directory already exists, we do nothing
        pass
    except:
        # Else we panic
        raise

def main():
    """ The entry point of the application.
    The function initializes the app, configures the settings, creates the network,
    trains and evaluates the model.
    """
    # Read the config json file
    config = {}
    config_path = os.fspath(Path(FLAGS.training_config_path).resolve())
    with open(config_path) as f:
        config = json.load(f)

    # Create the application.
    app_filename = os.fspath(Path(config['app_filename']).resolve())
    app = Application(app_filename=app_filename)
    # Startup the bridge to get data.
    node = app.nodes["pose_estimation_training_samples"]
    assert node is not None
    bridge = packages.ml.SampleAccumulator(node._node)
    app.start()

    try:
        # Get the dataset batch
        dataset = get_dataset(bridge, config)
        iterator = dataset.make_one_shot_iterator()
        data_dict = iterator.get_next()
        # Encoder input image
        input_image = data_dict[COLOR_IMAGE]
        # Adding Gaussian noise to input image
        # This acts as data augmentation tool and also stabilizes the weights of the network
        # at the beginning of the training.
        if (config['add_noise_to_image']):
            noise = tf.random_normal(shape=tf.shape(input_image), mean=0.0, stddev=0.01, dtype=tf.float32)
            input_image = tf.add(input_image, noise)

        # Reconstructed image to compute decoder loss
        reconstruct_image = data_dict[RECONSTRUCT_IMAGE]
        if (config['add_noise_to_image']):
            noise = tf.random_normal(shape=tf.shape(reconstruct_image), mean=0.0, stddev=0.01, dtype=tf.float32)
            reconstruct_image  = tf.add(reconstruct_image, noise)

        # Create the autoencoder network
        encoder = build_encoder(input_image, config['latent_space_size'],
            config['num_filter'], config['kernel_size_encoder'],
            config['strides'], config['batch_norm'], is_training = True)
        decoder = build_decoder(reconstruct_image, encoder, config['variational'],
            config['num_filter'], config['kernel_size_decoder'], config['strides'],
            config['loss'], config['bootstrap_ratio'], config['auxiliary_mask'],
            config['batch_norm'], is_training = True)
        ae = build_ae(encoder, decoder, config['norm_regularize'], config['variational'])
        train_op = build_train_op(ae, config['learning_rate'])
        global_step = tf.placeholder(tf.int32)

        # Setup logging summaries and checkpoints..
        tf.summary.image('Input_image', input_image, max_outputs = 1)
        tf.summary.image('GT reconstruct_image', reconstruct_image, max_outputs = 1)
        saver = tf.train.Saver()

        # Setup logging folders.
        ckpt_dir = os.path.join(Path(config['train_logdir']).resolve(), 'ckpts')
        ckpt_file = os.path.join(Path(ckpt_dir, 'model'))
        # Create directories if not loading from existing checkpoint
        if not config['checkpoint']:
            make_dir(ckpt_dir)

        # Create tensorflow session configuration
        session_config = tf.ConfigProto()
        session_config.gpu_options.per_process_gpu_memory_fraction = config['gpu_memory_usage']
        session_config.gpu_options.allow_growth = True

        # Wait until we get enough samples
        while True:
            sample_number = bridge.get_sample_count()
            if sample_number >= config['batch_size']:
                break
            time.sleep(config['sleep_duration'])
            print("waiting for samples samples: {}".format(sample_number))
        print("Starting training...")

        # Training loop.
        with tf.Session(config=session_config) as sess:
            merged_loss_summary = tf.summary.merge_all()
            summary_writer = tf.summary.FileWriter(ckpt_dir, sess.graph)
            output_node_name = 'encoder_output/BiasAdd'
            input_node_name = 'encoder_input'
            sess.run(tf.global_variables_initializer())
            tf.train.write_graph(sess.graph, ckpt_dir, 'graph.pb', as_text=False)

            if config['checkpoint']:
                saver.restore(sess, os.path.join(ckpt_dir, config['checkpoint']))
                print("Checkpoint Loaded - {}".format(config['checkpoint']))

                # Assumes the number of steps is at the end of the
                # checkpoint file (separated by a '-')
                init_step = int(config['checkpoint'].split("-")[-1]) + 1
            else:
                init_step = 0

            # Start training
            for step in range(init_step, config['training_steps']):
                if step % config['summary_every'] == 0:
                    loss = sess.run(merged_loss_summary)
                    summary_writer.add_summary(loss, step)
                else:
                    sess.run(train_op)
                tf.logging.info('step: {}, loss: {} '.format(step, ae.loss))
                # Save checkpoint
                if step % config['save_every'] == 0 or step == config['training_steps'] - 1:
                    saver.save(sess, ckpt_file, global_step=step)
                gc.collect()

        # Saving model.pb file of the last iteration
        # Note: This should not be called inside another tf Session
        frozen_file = os.path.join("{}-{}-frozen.pb".format(ckpt_file, config['training_steps'] - 1))
        freeze_graph.freeze_graph(
            input_graph=os.path.join(ckpt_dir, 'graph.pb'),
            input_saver="",
            input_binary=True,
            input_checkpoint=os.path.join("{}-{}".format(ckpt_file, config['training_steps'] - 1)),
            output_node_names=output_node_name,
            restore_op_name="save/restore_all",
            filename_tensor_name="save/Const:0",
            output_graph=frozen_file,
            clear_devices=True,
            initializer_nodes="")
        print("Saved frozen model at {}.".format(frozen_file))
        app.stop()

    except KeyboardInterrupt:
        print("Exiting due to keyboard interrupt")
        app.stop()


if __name__ == '__main__':
    main()
