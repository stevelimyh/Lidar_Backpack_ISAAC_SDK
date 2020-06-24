'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

"""Voice Command Training script

Trains a neural network which takes in Mel spectrogram, deltas of order 1 and order 2 as input and
outputs keyword-wise probabilities.

usage: training.py [-h] [-t TRAIN_DATASET_PATH]
                   [--validation_dataset_path VALIDATION_DATASET_PATH] [-n]
                   [--noise_profile_path NOISE_PROFILE_PATH] [--tmpdir TMPDIR]
                   [--logdir LOGDIR] [-o MODEL_OUTPUT_PATH]
                   [-k KEYWORDS_LIST [KEYWORDS_LIST ...]]
                   [--keyword_duration KEYWORD_DURATION]
                   [--training_epochs TRAINING_EPOCHS]
                   [--batch_size BATCH_SIZE]
                   [--minimum_noise_gain MINIMUM_NOISE_GAIN]
                   [--maximum_noise_gain MAXIMUM_NOISE_GAIN] [--restrictive]
                   [--learning_rate LEARNING_RATE] [--dropout DROPOUT]
                   [--checkpoint CHECKPOINT] [-e EPOCH_NUMBER]
                   [--gpu_memory_usage GPU_MEMORY_USAGE]
                   [--config_filename CONFIG_FILENAME]

Command line arguments:
  -h, --help            show this help message and exit

  -t TRAIN_DATASET_PATH
  --train_dataset_path TRAIN_DATASET_PATH
                        Path to the training dataset.

  --validation_dataset_path VALIDATION_DATASET_PATH
                        Path to the validation dataset.

  -n, --augment-noise   Enable noise augmentation for training. Default: disabled

  --noise_profile_path NOISE_PROFILE_PATH
                        Path to the noise profiles (wav files).

  --tmpdir TMPDIR       Path to a directory where the processed data and checkpoints are
                        temporarily stored. Default: '/tmp'

  --logdir LOGDIR       Path to directoy where training logs are stored for Tensorboard
                        usage. Default: '<tmpdir>/logs'

  -o MODEL_OUTPUT_PATH
  --model_output_path MODEL_OUTPUT_PATH
                        Path to directory where the trained model and metadata are stored.

  -k KEYWORDS_LIST [KEYWORDS_LIST ...]
  --keywords_list KEYWORDS_LIST [KEYWORDS_LIST ...]
                        List of keywords to be detected. Keywords can be separated by a
                        comma in the list.
                        Eg.: '-k carter,look,stop', '-k carter look -k stop'

  --keyword_duration KEYWORD_DURATION
                        Duration of keywords in seconds in the range [0.1, 1].
                        Default: 0.5

  --training_epochs TRAINING_EPOCHS
                        Number of epochs to run the training. Default: 100

  --batch_size BATCH_SIZE
                        Batch size used for training. Default: 32

  --minimum_noise_gain MINIMUM_NOISE_GAIN
                        Minimum noise gain applied during noise augmentation.
                        Default: 0.1

  --maximum_noise_gain MAXIMUM_NOISE_GAIN
                        Maximum noise gain applied during noise augmentation.
                        Default: 0.4

  --learning_rate LEARNING_RATE
  --lr LEARNING_RATE
                        Learning rate used for Adamax optimizer. Default: 1e-5

  --dropout DROPOUT     Dropout value used for training the model. Default: 0.3

  --checkpoint CHECKPOINT
                        Keras checkpoint to be loaded to continue training. This assumes
                        that the extracted features are available <tmpdir>/features/.
                        Defaults to not loading checkpoints and starting fresh.

  -e EPOCH_NUMBER, --epoch_number EPOCH_NUMBER
                        Epoch at which to start training when resuming from checkpoint.
                        Default: 0

  --gpu_memory_usage GPU_MEMORY_USAGE
                        Specified to limit the usage of gpu memory in the range [0, 1].
                        Default: 0 (no limit)

  --config_filename CONFIG_FILENAME
                        Path to load a json file with all the configuration parameters.
                        However, Command line arguments take the priority.
"""

import os
os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'    # so the IDs match nvidia-smi
os.environ['CUDA_VISIBLE_DEVICES'] = '0'    # Limit the GPU usage to gpu #0

import argparse
from collections import OrderedDict
import h5py
import json
from keras import backend as K
from keras.backend.tensorflow_backend import set_session
from keras.models import Model, load_model
from keras.layers.core import Activation, Dense, Dropout, Reshape, Permute
from keras.layers import Input
from keras.layers import CuDNNGRU
from keras.callbacks import ModelCheckpoint, TerminateOnNaN, TensorBoard, EarlyStopping
from keras.optimizers import Adamax
import logging
import numpy as np
from os.path import join
from sklearn.metrics import roc_curve
import tensorflow as tf
from tensorflow.python.framework import graph_util, graph_io
from tensorflow.python.tools import freeze_graph
import time
from utils import create_dir, DataConfig, preprocess_data, remove_create_dir


# Dict of arguments that will be parsed from config and command line
args = None
# Default values of arguments
kArgsDefaults = {
    'config_filename': 'training.config.json',
    'augment_noise': False,
    'tmpdir': '/tmp',
    'model_output_path': 'model',
    'keyword_duration': '0.5',
    'training_epochs': 100,
    'batch_size': 32,
    'minimum_noise_gain': 0.1,
    'maximum_noise_gain': 0.4,
    'restrictive': False,
    'learning_rate': 1e-5,
    'dropout': 0.3,
    'epoch_number': 0
}
# Name of the checkpoint file
kCheckpointFileName = 'isaac_vcd_model'
# Name of the generated metadata file
kMetadataFileName = 'isaac_vcd_model.metadata.json'
# Name format of the intermediate keras models
kCheckpointAllFileName = 'isaac_vcd_model_{epoch:02d}_{loss:4f}_{val_loss:.4f}_{val_acc:.4f}.h5'
# Op names
kOutputOpName = 'output_node'


def create_network(num_classes):
    """RNN Network for audio classification

    Args:
        num_classes: Number of classification classes.
    Returns:
        A keras model.
    """
    input_tensor = Input(shape=(None, None, DataConfig.kNumFeatures), name='input_node')

    features = Permute([1,3,2])(input_tensor)
    flat = Reshape((-1, DataConfig.kNumFeatures * DataConfig.kNumMels))(features)

    gru1 = CuDNNGRU(196, return_sequences=True)(flat)
    act1 = Activation('tanh')(gru1)
    drop1 = Dropout(args.dropout)(act1)

    gru2 = CuDNNGRU(128, return_sequences=True)(drop1)
    act2 = Activation('tanh')(gru2)
    drop2 = Dropout(args.dropout)(act2)

    gru3 = CuDNNGRU(64)(drop2)
    act3 = Activation('sigmoid')(gru3)

    output_tensor = Dense(num_classes, activation='sigmoid', name='output_layer')(act3)

    model = Model(input_tensor, output_tensor)
    return model


def load_local_dataset(path):
    """Load a HDF5 dataset.

    Args:
        path: The hdf5 dataset path.
    Returns:
        numpy arrays for input audio and labels.
    """
    with h5py.File(path, 'r') as hf:
        input_audio = np.array(hf.get('input'))
        labels = np.array(hf.get('labels'))

    return input_audio, labels


def train_model(model, input_audio, labels, input_audio_validation, labels_validation):
    """Train the network using input_audio and labels as training dataset."""
    # Generate class weights
    class_weights = labels.sum(axis=0).max() / labels.sum(axis=0)
    class_weights = {i: weight for i, weight in enumerate(class_weights)}

    # Create the loss function.
    loss = 'binary_crossentropy'

    # Create the optimizer
    optimizer = Adamax(lr=args.learning_rate)

    # Setup callback for saving checkpoint.
    checkpoint_file_path = join(args.tmpdir, kCheckpointFileName + '.h5')
    checkpoint = ModelCheckpoint(
        checkpoint_file_path, monitor='val_loss', verbose=1, save_best_only=True, mode='auto')
    logging.debug('Saving best keras model at {}'.format(checkpoint_file_path))

    # Setup callback for saving all intermediate models
    remove_create_dir(join(args.tmpdir, 'models'))
    checkpoint_all_file_path = join(args.tmpdir, 'models', kCheckpointAllFileName)
    checkpoint_all = ModelCheckpoint(
        checkpoint_all_file_path, monitor='val_loss', save_best_only=False, mode='auto')
    logging.debug('Saving keras models after every epoch at {}'.format(join(args.tmpdir, 'models')))

    earlystop = EarlyStopping(monitor='val_loss', patience=20, verbose=0, mode='auto')

    # Setup logging with Tensorboard.
    create_dir(args.logdir)
    timestamp = time.strftime('%Y%m%d-%H%M%S')
    log_dir = join(args.logdir, '{}'.format(timestamp))
    tensorboard = TensorBoard(log_dir=log_dir, batch_size=args.batch_size)
    logging.debug('Tensorboard logs are available at {}'.format(log_dir))

    logging.debug('Started training with input audio shape={}, labels shape={}'.format(
        input_audio.shape, labels.shape))
    callbacks_list = [checkpoint, checkpoint_all, TerminateOnNaN(), tensorboard]

    # Prepare the network for training
    model.compile(loss=loss, optimizer=optimizer, metrics=['accuracy'])

    model.fit(
        x=input_audio,
        y=labels,
        batch_size=args.batch_size,
        epochs=args.training_epochs,
        callbacks=callbacks_list,
        validation_data=(input_audio_validation, labels_validation),
        class_weight=class_weights)

    return model


def continue_train_model(model, input_audio, labels, input_audio_validation, labels_validation):
    """Train the network from a checkpoint using input_audio and labels as training dataset."""
    # Generate class weights
    class_weights = labels.sum(axis=0).max() / labels.sum(axis=0)
    class_weights = {i: weight for i, weight in enumerate(class_weights)}

    # Create the loss function.
    loss = 'binary_crossentropy'

    # Create the optimizer
    optimizer = Adamax(lr=args.learning_rate)

    # Setup callback for saving checkpoint.
    checkpoint_file_path = join(args.tmpdir, kCheckpointFileName + '.h5')
    checkpoint = ModelCheckpoint(
        checkpoint_file_path, monitor='val_loss', verbose=1, save_best_only=True, mode='auto')
    logging.debug('Saving best keras model at {}'.format(checkpoint_file_path))

    # Setup callback for saving all intermediate models
    remove_create_dir(join(args.tmpdir, 'models'))
    checkpoint_all_file_path = join(args.tmpdir, 'models', kCheckpointAllFileName)
    checkpoint_all = ModelCheckpoint(
        checkpoint_all_file_path, monitor='val_loss', save_best_only=False, mode='auto')
    logging.debug('Saving keras models after every epoch at {}'.format(join(args.tmpdir, 'models')))

    earlystop = EarlyStopping(monitor='val_loss', patience=20, verbose=0, mode='auto')

    # Setup logging with Tensorboard.
    create_dir(args.logdir)
    timestamp = time.strftime('%Y%m%d-%H%M%S')
    log_dir = join(args.logdir, '{}'.format(timestamp))
    tensorboard = TensorBoard(log_dir=log_dir, batch_size=args.batch_size)
    logging.debug('Tensorboard logs are available at {}'.format(log_dir))

    logging.debug('Started training with input audio shape={}, labels shape={}'.format(
        input_audio.shape, labels.shape))
    callbacks_list = [checkpoint, checkpoint_all, TerminateOnNaN(), tensorboard]

    model.fit(
        x=input_audio,
        y=labels,
        initial_epoch = args.epoch_number,
        batch_size=args.batch_size,
        epochs=args.training_epochs,
        callbacks=callbacks_list,
        validation_data=(input_audio_validation, labels_validation),
        class_weight=class_weights)


def find_optimal_threshold(target, predicted, label_id):
    """Find the optimal probability threshold for a classification model.

    Args:
       target: Matrix with dependent or target data, where rows are observations.
       predicted: Matrix with predicted data, where rows are observations.
       label_id: Id of class or label for which probability threshold is to be computed.
    Returns:
       optimal cutoff value.
    """
    fpr, tpr, threshold = roc_curve(target, predicted, pos_label=label_id, drop_intermediate=False)
    return threshold[(tpr - fpr).argsort()[-1]].astype(float)


def generate_metadata(tf_config, input_audio, labels):
    """Generates metadata file containing sample rate and thresholds of each keyword"""
    K.clear_session()
    sess = tf.Session(config=tf_config)
    set_session(sess)
    K.set_learning_phase(0)

    model_file = join(args.tmpdir, kCheckpointFileName + '.h5')
    model = load_model(model_file)

    prediction_probabilities = model.predict(input_audio, batch_size=args.batch_size)
    label_ids = np.where(labels)[1]
    sorted_indices = label_ids.argsort()

    prediction_probabilities = prediction_probabilities[sorted_indices]
    label_ids = label_ids[sorted_indices]
    class_range = np.concatenate(([0], np.cumsum(labels.sum(axis=0)))).astype(int)

    # Metadata
    num_classes = len(args.keywords_list)
    thresholds = [None] * num_classes

    max_range = labels.sum(axis=0).min().astype(int) * (num_classes - 1)

    # Compute optimal threshold for each class
    for i in range(num_classes):
        total = min(class_range[i + 1] - class_range[i], max_range)
        chosen_indices = np.random.choice(
            np.arange(class_range[i], class_range[i + 1]), total, replace=False)
        target_labels = [label_ids[chosen_indices]]
        predictions = [prediction_probabilities[chosen_indices, i]]

        split = np.round(total / (num_classes - 1)).astype(int)
        for j in range(num_classes):
            if j != i:
                chosen_indices = np.random.choice(
                    np.arange(class_range[j], class_range[j + 1]), min(split, total), replace=False)
                total -= split
                target_labels.append(label_ids[chosen_indices])
                predictions.append(prediction_probabilities[chosen_indices, i])

        target_labels = np.concatenate(target_labels)
        predictions = np.concatenate(predictions)
        thresholds[i] = find_optimal_threshold(target_labels, predictions, i)

    # Prepare and save the metadata
    input_tensor = model.inputs[0]
    input_layer_name = input_tensor.name.split(':')[0].split('/')[0]
    input_tensor_shape = [1, DataConfig.kWindowLength, DataConfig.kNumMels, DataConfig.kNumFeatures]

    metadata = OrderedDict(
        [('<feature_extraction_node_name>', OrderedDict(
            [('isaac.audio.VoiceCommandFeatureExtraction', OrderedDict(
                [('sample_rate', DataConfig.kSampleRate),
                 ('fft_length', DataConfig.kFftLength),
                 ('num_mels', DataConfig.kNumMels),
                 ('hop_size', DataConfig.kHopLength),
                 ('window_length', DataConfig.kWindowLength),
                 ('num_classes', num_classes),
                 ('classes', args.keywords_list)]))
            ])),
         ('<tensorflow_inference_node_name>', OrderedDict(
            [('isaac.ml.TensorflowInference', OrderedDict(
                [('input_tensor_info',
                    [OrderedDict([('ops_name', input_layer_name),
                                  ('index', 0),
                                  ('dims', input_tensor_shape)])
                    ]),
                 ('output_tensor_info',
                    [OrderedDict(
                        [('ops_name', kOutputOpName),
                         ('index', 0),
                         ('dims', [1, num_classes])])
                    ])
                ]))
            ])),
         ('<command_construction_node_name>', OrderedDict(
            [('isaac.audio.VoiceCommandConstruction', OrderedDict(
                [('num_classes', num_classes),
                 ('classes', args.keywords_list),
                 ('thresholds', list(thresholds))]))
            ]))
        ])

    create_dir(args.model_output_path)
    with open(join(args.model_output_path, kMetadataFileName), 'w') as f:
        json.dump(metadata, f, indent=2)


def convert_keras_to_tensorflow(tf_config):
    """Convert the trained model from Keras to frozen Tensorflow graph"""
    K.clear_session()
    sess = tf.Session(config=tf_config)
    set_session(sess)
    K.set_learning_phase(0)

    model_file = join(args.tmpdir, kCheckpointFileName + '.h5')
    try:
        model = load_model(model_file)
    except ValueError as err:
        logging.error(('Input file specified ({}) only holds the weights, ' +
                       'and not the model definition.').format(model_file))
        raise err

    output_op = tf.identity(model.outputs[0], name=kOutputOpName)
    sess = K.get_session()
    constant_graph = graph_util.convert_variables_to_constants(sess, sess.graph.as_graph_def(),
                                                               [kOutputOpName])

    create_dir(args.model_output_path)
    graph_io.write_graph(
        constant_graph, args.model_output_path, kCheckpointFileName + '.pb', as_text=False)

    logging.info('===============================================================')
    logging.info('Please note down the below information. It will be required in ' +
                 'configuring TensorflowInference component.')
    input_tensor = model.inputs[0]
    logging.info('Input tensor info:')
    logging.info('  name: {}'.format(input_tensor.name.split(':')[0].split('/')[0]))
    logging.info('  index: 0')
    logging.info('  dims: {}'.format([1 if d is None else d for d in input_tensor.shape.as_list()]))
    logging.info('Output tensor info:')
    logging.info('  name: {}'.format(kOutputOpName))
    logging.info('  index: 0')
    logging.info('  dims: {}'.format([1, len(args.keywords_list)]))
    logging.info('Saved the frozen graph (ready for inference) at {}/{}'.format(
        os.path.abspath(args.model_output_path), kCheckpointFileName + '.pb'))
    logging.info('Saved the metadata file at {}/{}'.format(os.path.abspath(args.model_output_path),
                                                           kMetadataFileName))
    logging.info('===============================================================')


def check_args(key, value1, value2=None, value3=None):
    """Check if a value is available and print an error if it is missing"""
    value = value1 or value2 or value3
    if value is None:
        raise ValueError('Missing configuration for {}'.format(key))
    return value


def parse_config():
    """Parse the config.json file and compare with the command line arguments to populate the args.
    Command line arguments are prioritised over the config file.
    """
    global args
    config = json.loads('{}')
    config_filename = args.config_filename or os.path.exists(kArgsDefaults['config_filename'])
    if config_filename:
        with open(config_filename) as f:
            config = json.load(f)

    args.train_dataset_path = check_args('train_dataset_path', args.train_dataset_path,
                                         config.get('train_dataset_path'))
    args.validation_dataset_path = check_args('validation_dataset_path',
                                              args.validation_dataset_path,
                                              config.get('validation_dataset_path'))
    args.augment_noise = check_args('augment_noise', args.augment_noise,
                                    config.get('augment_noise'), kArgsDefaults['augment_noise'])
    args.noise_profile_path = args.noise_profile_path or config.get('noise_profile_path')
    args.tmpdir = check_args('tmpdir', args.tmpdir, config.get('tmpdir'), kArgsDefaults['tmpdir'])
    args.tmpdir = join(args.tmpdir, 'isaac_voice_command_training')

    args.logdir = check_args('logdir', args.logdir, config.get('logdir'), join(args.tmpdir, 'logs'))
    args.model_output_path = check_args('model_output_path', args.model_output_path,
                                        config.get('model_output_path'),
                                        kArgsDefaults['model_output_path'])
    args.keywords_list = ','.join(args.keywords_list).split(',') if args.keywords_list else None
    args.keywords_list = check_args('keywords_list', args.keywords_list,
                                    config.get('keywords_list'))
    args.keywords_list = [keyword.lower() for keyword in args.keywords_list]
    args.keywords_list.sort()
    args.keywords_list += ['unknownkeywords']

    args.keyword_duration = check_args('keyword_duration', args.keyword_duration,
                                       config.get('keyword_duration'),
                                       kArgsDefaults['keyword_duration'])
    args.keyword_duration = int(args.keyword_duration * 1000)

    args.training_epochs = check_args('training_epochs', args.training_epochs,
                                      config.get('training_epochs'),
                                      kArgsDefaults['training_epochs'])
    args.batch_size = check_args('batch_size', args.batch_size, config.get('batch_size'),
                                 kArgsDefaults['batch_size'])
    args.minimum_noise_gain = check_args('minimum_noise_gain', args.minimum_noise_gain,
                                         config.get('minimum_noise_gain'),
                                         kArgsDefaults['minimum_noise_gain'])
    args.maximum_noise_gain = check_args('maximum_noise_gain', args.maximum_noise_gain,
                                         config.get('maximum_noise_gain'),
                                         kArgsDefaults['maximum_noise_gain'])
    args.restrictive = check_args('restrictive', args.restrictive, config.get('restrictive'),
                                  kArgsDefaults['restrictive'])
    args.learning_rate = check_args('learning_rate', args.learning_rate,
                                    config.get('learning_rate'), kArgsDefaults['learning_rate'])
    args.dropout = check_args('dropout', args.dropout, config.get('dropout'),
                              kArgsDefaults['dropout'])

    args.checkpoint = args.checkpoint or config.get('checkpoint')
    args.epoch_number = check_args('epoch_number', args.epoch_number, config.get('epoch_number'),
                                   kArgsDefaults['epoch_number'])
    args.gpu_memory_usage = args.gpu_memory_usage or config.get('gpu_memory_usage')


def parse_command_line_arguments():
    """Define and parse the command line arguments of the binary into args"""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-t',
        '--train_dataset_path',
        dest='train_dataset_path',
        help='Where the training dataset is stored.')
    parser.add_argument(
        '--validation_dataset_path',
        dest='validation_dataset_path',
        help='Where the validation dataset is stored.')
    parser.add_argument(
        '-n',
        '--augment-noise',
        action='store_true',
        dest='augment_noise',
        help='Enable noise augmentation for training.')
    parser.add_argument(
        '--noise_profile_path',
        dest='noise_profile_path',
        help='Where the noise profiles are stored.')
    parser.add_argument(
        '--tmpdir',    # default='/tmp',
        dest='tmpdir',
        help='Where the processed data and checkpoints are temporarily stored. Default: /tmp')
    parser.add_argument(
        '--logdir',
        dest='logdir',
        help='Where training logs are stored for Tensorboard usage. Default is <tmpdir>/logs')
    parser.add_argument(
        '-o',
        '--model_output_path',    # default='model',
        dest='model_output_path',
        help='Where the trained model and metadata are stored. Default: <current_dir>/model')
    parser.add_argument(
        '-k',
        '--keywords_list',
        nargs='+',
        dest='keywords_list',
        help='List of keywords to be detected.')
    parser.add_argument(
        '--keyword_duration',
        type=float,    # default='500ms',
        dest='keyword_duration',
        help='Duration of keywords in seconds. Range is [0.1, 1]. Default: 0.5')
    parser.add_argument(
        '--training_epochs',
        type=int,    # default=100,
        dest='training_epochs',
        help='Number of epochs to run the training. Default: 100')
    parser.add_argument(
        '--batch_size',
        type=int,    # default=32,
        dest='batch_size',
        help='Batch size used for training. Default: 32')
    parser.add_argument(
        '--minimum_noise_gain',
        type=float,    # default=0.1,
        dest='minimum_noise_gain',
        help='Minimum noise gain applied during noise augmentation. Default: 0.1')
    parser.add_argument(
        '--maximum_noise_gain',
        type=float,    # default=0.4,
        dest='maximum_noise_gain',
        help='Maximum noise gain applied during noise augmentation. Default: 0.4')
    parser.add_argument(
        '--restrictive',
        action='store_true',
        dest='restrictive',
        help='Train the model limited number of speaker. Disabled by default.')
    parser.add_argument(
        '--learning_rate',
        '--lr',
        type=float,    # default=1e-5,
        dest='learning_rate',
        help='Learning rate used for Adamax optimizer. Default: 1e-5')
    parser.add_argument(
        '--dropout', dest='dropout', help='Dropout value used for training')
    parser.add_argument(
        '--checkpoint',
        dest='checkpoint',
        help='Keras checkpoint to be loaded to continue training. ' +
        'Defaults to not loading checkpoints.')
    parser.add_argument(
        '-e',
        '--epoch_number',
        type=int,    # default=0,
        dest='epoch_number',
        help='Epoch at which to start training when resuming from checkpoint. Default: 0')
    parser.add_argument(
        '--gpu_memory_usage',
        type=float,
        dest='gpu_memory_usage',
        help='Specified to limit the usage of gpu memory. Default: 0 (no limit)')

    parser.add_argument(
        '--config_filename',    # default='training.config.json',
        dest='config_filename',
        help='Path to json file with configuration parameters. ' +
        'However command line arguments are prioritised. Default: training.config.json if exists.')

    global args
    args = parser.parse_args()


def main():
    """Voice Command Training Application

    End-to-end app for augmenting the datasets, extracting the features, training the network,
    validating the model, computing the optimal probability thresholds for each class, generate
    the metadata and save the model as tensorflow graph.
    """

    logging.basicConfig(
        format='%(asctime)s %(levelname)s: %(message)s',
        datefmt='%m/%d/%Y %I:%M:%S %p',
        level=logging.DEBUG)
    # Parse the command line arguments into args
    parse_command_line_arguments()

    # Parse the config.json file
    parse_config()
    logging.debug('Parse arguments: {}'.format(args))

    # Limit the GPU memory usage.
    tf_config = tf.ConfigProto()
    if args.gpu_memory_usage:
        tf_config.gpu_options.per_process_gpu_memory_fraction = args.gpu_memory_usage
    else:
        tf_config.gpu_options.allow_growth = True
    sess = tf.Session(config=tf_config)
    set_session(sess)

    if args.checkpoint is None:
        # Augment the dataset and extract the features in preprocessing stage
        preprocess_data({'train': args.train_dataset_path,
                         'valid': args.validation_dataset_path},
                        args.tmpdir,
                        args.augment_noise,
                        args.noise_profile_path,
                        {keyword: i
                         for i, keyword in enumerate(args.keywords_list)},
                        args.keyword_duration,
                        {'min': args.minimum_noise_gain,
                         'max': args.maximum_noise_gain},
                        cpu_usage=1)

        # Create the network.
        rnn_network = create_network(len(args.keywords_list))
    else:
        # Compute the keyword window length
        keyword_length = int(args.keyword_duration * DataConfig.kSampleRate / 1000)
        DataConfig.kWindowLength = int(
            np.floor((keyword_length - DataConfig.kFftLength + DataConfig.kHopLength) /
                     DataConfig.kHopLength))

        # Load the checkpoint model
        try:
            rnn_model = load_model(args.checkpoint)
        except ValueError as err:
            logging.error(('Input file specified ({}) only holds the weights, ' +
                           'and not the model definition.').format(args.checkpoint))
            raise err

    # Load training dataset
    input_audio, labels = load_local_dataset(join(args.tmpdir, 'features', 'train.hdf5'))

    # Load validation dataset
    input_audio_validation, labels_validation = load_local_dataset(
        join(args.tmpdir, 'features', 'valid.hdf5'))

    if args.checkpoint:
        # Continue training from checkpoint
        continue_train_model(rnn_model, input_audio, labels, input_audio_validation,
                             labels_validation)
    else:
        # Train the model
        train_model(rnn_network, input_audio, labels, input_audio_validation, labels_validation)

    # Generate the metadata
    generate_metadata(tf_config, input_audio_validation, labels_validation)

    # Convert the keras model to tensorflow
    convert_keras_to_tensorflow(tf_config)

    del input_audio, labels
    del input_audio_validation, labels_validation


if __name__ == '__main__':
    main()
