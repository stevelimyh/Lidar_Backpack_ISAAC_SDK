'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

from engine.pyalice import *
from keras import backend as K
from keras.optimizers import Adam
from freespace_dnn_training_models import UNet
from pathlib import Path
from tensorflow import keras

import packages.freespace_dnn.apps
import json
import numpy as np
import os
import packages.ml
import shutil
import tensorflow as tf
import time

# Command line flags:
flags = tf.app.flags
FLAGS = flags.FLAGS
flags.DEFINE_string('training_config_path',
                    'packages/freespace_dnn/apps/freespace_dnn_training.config.json',
                    'Relative path to the config file containing training configurations')
# Dictionary to store the parsed training configurations
config = {}


class TimeHistory(tf.train.SessionRunHook):
    """  A SessionHook class to record the durations of each iteration of the optimizer

    Attributes:
    times: list of time intervals in seconds, populated on each run
    iter_time_start: the timestamp at the start of the interation

    Methods:
    begin: initializes the list of time intervals
    before_run: records the timestamp at the beginning of each iteration
    after_run: records the difference in timestamps from the start to the end of the iteration
    """

    def begin(self):
        self.times = []

    def before_run(self, run_context):
        self.iter_time_start = time.time()

    def after_run(self, run_context, run_values):
        self.times.append(time.time() - self.iter_time_start)


def create_device_list(num_gpus):
    """ Creates a list of GPU devices which we can make available for training

    Args:
    num_gpus: the number of GPU's we are making available

    Returns:
    The list of GPU devices in the format expected by tf.strategy
    """
    prefix = "/device:GPU:"
    device_list = []
    for i in range(config['num_gpus']):
        device = prefix + str(i)
        device_list.append(device)
    return device_list


def dice_coefficient(y_true, y_pred):
    """ Computes the dice coefficient value.
    This is a direct indication of the coverage of positively predicted pixels.

    Args:
    y_true: the ground truth labels
    y_pred: the predicted labels

    Returns:
    The dice loss for the given batch of predictions
    """
    smooth = tf.keras.backend.epsilon()
    y_true = K.flatten(y_true)
    y_pred = K.flatten(y_pred)
    intersection = K.sum(y_true * y_pred)
    return (2. * intersection + smooth) / (K.sum(y_true) + K.sum(y_pred) + smooth)


def dice_coefficient_loss(y_true, y_pred):
    """ Computes the dice loss.
    This would have the inverse trend as the dice coefficient value, in the sense that the higher
    the intersection over union, the lesser the loss should be.

    Args:
    y_true: the ground truth labels
    y_pred: the predicted labels

    Returns:
    The dice loss for the given batch of predictions
    """
    return 1 - dice_coefficient(y_true, y_pred)


def bce_dice_loss(y_true, y_pred):
    """ Computes custom weighted loss combining binary cross entropy and dice losses.

    Args:
    y_true: the ground truth labels
    y_pred: the predicted labels

    Returns:
    The loss for the given batch of predictions
    """
    reshaped_labels = tf.reshape(y_true, shape=[config['batch_size'], -1])
    reshaped_logits = tf.reshape(y_pred, shape=[config['batch_size'], -1])
    bce_loss = K.binary_crossentropy(reshaped_labels, reshaped_logits)
    bce_loss = tf.reduce_mean(bce_loss * config['class_weight'])
    dice_loss = dice_coefficient_loss(y_true, y_pred)
    loss = (0.7 * bce_loss) + (0.3 * dice_loss)
    return loss


def cross_entropy_loss(y_true, y_pred):
    """ Computes categorical cross entropy loss for multi-class segmentation.

    Args:
    y_true: the ground truth labels
    y_pred: the predicted labels

    Returns:
    The loss for the given batch of predictions
    """
    reshaped_labels = tf.reshape(y_true, shape=[config['batch_size'], -1])
    reshaped_logits = tf.reshape(y_pred, shape=[config['batch_size'], -1])
    ce_loss = K.categorical_crossentropy(reshaped_labels, reshaped_logits)
    ce_loss = tf.reduce_mean(ce_loss)
    return ce_loss


def get_generator(bridge):
    """ Creates a training sample generator.

    Args:
    bridge: the isaac sample accumulator node which we will acquire samples from

    Returns:
    A generator function which yields a single training example.
    """

    def _generator():
        iterations_without_samples = 0
        while True:
            # Try to acquire a sample of size equal to batch size.
            # If none are available, then wait for a bit so we do not spam the app.
            sample = bridge.acquire_samples(config['batch_size'])
            if not sample:
                iterations_without_samples += 1
                if (iterations_without_samples >=
                    (config['timeout_duration'] / config['sleep_duration'])):
                    # The timeout has passed, assume bridge has stalled and stop the generator.
                    raise StopIteration
                else:
                    # Wait for a bit so we do not spam the app, then try again.
                    time.sleep(config['sleep_duration'])
                    continue
            # Reset missing sample count if we get samples
            iterations_without_samples = 0
            # We should get samples with two tensors (the input image and labels).
            assert len(sample) >= 1
            assert len(sample[0]) == 2
            num_samples = min(len(sample), config['batch_size'])
            for i in range(num_samples):
                yield sample[i][0], sample[i][1]

    return _generator


def get_dataset(bridge, num_classes):
    """ Create a tf.data dataset which yields batches of samples for training.

    Args:
    bridge: the isaac sample accumulator node which we will acquire samples from

    Returns:
    A tf.data dataset which yields batches of training examples.
    """

    dataset = tf.data.Dataset.from_generator(
        generator=get_generator(bridge),
        output_types=(tf.float32, tf.float32),
        output_shapes=((None, None, 3), (None, None, num_classes)))
    dataset = dataset.batch(batch_size=config['batch_size'])
    return dataset


def make_dir(path, load_from_checkpoint):
    """ Clears a directory and creates it on disk.

    Args:
    path: The path on disk where the directory should be located.
    """
    # If we don't want to load from existing checkpoint, delete the folder first
    if not load_from_checkpoint:
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


def model_function(features, labels, mode):
    """ Defines the operations for different modes in which the estimator is run.
    The predictions and loss are obtained as a common first step.
    In TRAIN mode, we use the loss for backpropagation.
    In EVAL mode, we calculate the accuracy and IoU score of the prediction.

    Args:
    features: the input RGB image
    labels: the predicted labels
    mode: the mode in which the estimator is currently running

    Returns:
    The estimator spec which defines the operation to perform
    """
    # Get output and calculate loss
    rows = config['rows']
    cols = config['cols']
    num_classes = config['num_classes']
    model = UNet(input_size=(rows, cols, 3), num_class=num_classes)
    logits = model(features)
    loss = bce_dice_loss(labels, logits)
    if (num_classes > 1):
        loss = cross_entropy_loss(labels, logits)
    # Define functions for train mode
    if mode == tf.estimator.ModeKeys.TRAIN:
        # Define the optimizer
        optimizer = tf.train.AdamOptimizer(learning_rate=config['learning_rate'])
        tf.summary.image('input_image', features, max_outputs=1)
        # Publish the images to tensorboard
        # tf.summary.image can only accept images of channel size 1, 3 or 4
        # Since we could have more channels while performing multi-class segmentation, we publish
        # each slice of the tensor in the channel dimension separately.
        if (num_classes > 1):
            for channel_index in range(num_classes):
                label_slice = tf.slice(labels, [0, 0, 0, channel_index], [1, rows, cols, 1])
                logits_slice = tf.slice(logits, [0, 0, 0, channel_index], [1, rows, cols, 1])
                tf.summary.image('ground_truth_' + str(channel_index), label_slice, max_outputs=1)
                tf.summary.image('prediction_' + str(channel_index), logits_slice, max_outputs=1)
        else:
            tf.summary.image('ground_truth', labels, max_outputs=1)
            tf.summary.image('predictions', logits, max_outputs=1)
        return tf.estimator.EstimatorSpec(
            mode=tf.estimator.ModeKeys.TRAIN,
            loss=loss,
            train_op=optimizer.minimize(loss, tf.train.get_or_create_global_step()))
    # Define functions for evaluation mode
    if mode == tf.estimator.ModeKeys.EVAL:
        accuracy = tf.metrics.accuracy(labels=labels, predictions=logits)
        iou_score = tf.metrics.mean_iou(labels=labels, predictions=logits, num_classes=2)
        metrics = {'accuracy': accuracy, 'iou_score': iou_score}
        tf.summary.scalar('accuracy', accuracy[0])
        tf.summary.scalar('iou_score', iou_score[1])
        return tf.estimator.EstimatorSpec(
            mode=tf.estimator.ModeKeys.EVAL, loss=loss, eval_metric_ops=metrics)


def train_and_evaluate():
    """ The entry point of the application.
     The function initializes the app, configures the settings, creates the estimator,
     trains and evaluates the model.
    """
    # Read the config json file
    config_path = os.fspath(Path(FLAGS.training_config_path).resolve())
    with open(config_path) as f:
        global config
        config = json.load(f)
    # Create the application.
    app_filename = os.fspath(Path(config['app_filename']).resolve())
    app = Application(app_filename=app_filename)
    # Startup the bridge to get data.
    node = app.nodes["sample_accumulator"]
    assert node is not None
    bridge = packages.ml.SampleAccumulator(node._node)
    app.start()
    try:
        # Define the directories for checkpoints
        checkpoint_dir = os.fspath(Path(config['train_logdir']).resolve())
        # Compute the number of repetitions
        num_iterations = int((config['data_points'] / config['batch_size']) / config['num_gpus'])
        # Initialize time hook
        time_hist = TimeHistory()
        # Create the directory for the model checkpoint directory
        make_dir(checkpoint_dir, config['load_from_checkpoint'])
        # Get list of GPU devices
        device_list = create_device_list(config['num_gpus'])
        strategy = tf.distribute.MirroredStrategy(
            devices=device_list) if config['use_mirrored_strategy'] else None
        # Create session configurations
        session_config = tf.ConfigProto()
        session_config.gpu_options.per_process_gpu_memory_fraction = config['gpu_memory_usage']
        session_config.gpu_options.allow_growth = True
        session_config.allow_soft_placement = True
        # Create run configurations
        run_config = tf.estimator.RunConfig(
            session_config=session_config,
            train_distribute=strategy,
            model_dir=checkpoint_dir,
            keep_checkpoint_max=10,
            save_checkpoints_steps=config['save_every'],
            save_summary_steps=1,
            log_step_count_steps=1)
        # Create the estimator
        estimator = tf.estimator.Estimator(model_function, config=run_config)
        # Create specs for train and evaluation
        train_spec = tf.estimator.TrainSpec(
            input_fn=lambda: get_dataset(bridge, config['num_classes']),
            max_steps=num_iterations,
            hooks=[time_hist])
        eval_spec = tf.estimator.EvalSpec(
            input_fn=lambda: get_dataset(bridge, config['num_classes']),
            steps=config['steps_per_eval'],
            throttle_secs=1,
            start_delay_secs=1)
        # Make sure we have a few samples before we initiate training
        while True:
            sample_number = bridge.get_sample_count()
            if sample_number >= config['initial_sample_count']:
                break
            time.sleep(config['sleep_duration'])
            print("Number of samples in sample buffer: ", sample_number)
        # Run the training and evaluation
        tf.estimator.train_and_evaluate(estimator, train_spec, eval_spec)
        total_time = sum(time_hist.times)
        print("Total time: ", total_time, " seconds")
    except KeyboardInterrupt:
        print("Exiting due to keyboard interrupt")
    except tf.errors.OutOfRangeError:
        print("Exiting due to training data stall")
    app.stop()


if __name__ == '__main__':
    train_and_evaluate()
