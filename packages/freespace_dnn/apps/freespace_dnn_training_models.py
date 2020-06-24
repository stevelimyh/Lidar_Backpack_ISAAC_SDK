'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

'''
This code file was inspired by: helper_functions.py at
https://github.com/MrGiovanni/UNetPlusPlus/blob/master/helper_functions.py by Zongwei Zhou
under the following license:

MIT License

Copyright (c) 2018 Zongwei Zhou

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

from keras import backend as K
from tensorflow.keras.layers import Dense, Dropout, Activation
from tensorflow.keras.layers import Input, Conv2D, concatenate, Conv2DTranspose
from tensorflow.keras.layers import MaxPooling2D, GlobalAveragePooling2D
from tensorflow.keras.models import Model
from tensorflow.keras.regularizers import l2
import tensorflow as tf

import numpy as np

kActivation = "relu"


def common_unit(input_tensor, stage, filters, kernel_size=3):
    """ Defines the stack of operations that are common to most of the UNet layers.

    Args:
    input_tensor: tensor input from the previous layer
    stage: the index of the stage of operation, only used for naming the operations
    filters: the dimensionality of the output space
    kernel_size: specifies the height and width of the 2D convolution window

    Returns:
    The output of the sequence of operations
    """
    x = Conv2D(
        filters, (kernel_size, kernel_size),
        activation=kActivation,
        name='conv' + stage + '_1',
        kernel_initializer='he_normal',
        padding='same',
        kernel_regularizer=l2(1e-4))(input_tensor)
    x = Conv2D(
        filters, (kernel_size, kernel_size),
        activation=kActivation,
        name='conv' + stage + '_2',
        kernel_initializer='he_normal',
        padding='same',
        kernel_regularizer=l2(1e-4))(x)
    return x


def UNet(input_size=(256, 512, 3), num_class=1):
    """ Defines the UNet semantic segmentation network architecture.
    Original paper: https://arxiv.org/abs/1505.04597

    Args:
    input_size: the size of the RGB input image
    num_class: the number of output classes

    Returns:
    The last layer of output, defining the probability of each pixel belonging to the class
    """
    filters = [32, 64, 128, 256, 512]
    axis = 3

    img_input = Input(input_size, name='input')

    conv1 = common_unit(img_input, stage='1', filters=filters[0])
    pool1 = MaxPooling2D((2, 2), strides=(2, 2), name='pool1')(conv1)

    conv2 = common_unit(pool1, stage='2', filters=filters[1])
    pool2 = MaxPooling2D((2, 2), strides=(2, 2), name='pool2')(conv2)

    conv3 = common_unit(pool2, stage='3', filters=filters[2])
    pool3 = MaxPooling2D((2, 2), strides=(2, 2), name='pool3')(conv3)

    conv4 = common_unit(pool3, stage='4', filters=filters[3])
    pool4 = MaxPooling2D((2, 2), strides=(2, 2), name='pool4')(conv4)

    conv5 = common_unit(pool4, stage='5', filters=filters[4])

    up6 = Conv2DTranspose(filters[3], (2, 2), strides=(2, 2), name='up6', padding='same')(conv5)
    conv6 = concatenate([up6, conv4], name='merge4', axis=axis)
    conv6 = common_unit(conv6, stage='6', filters=filters[3])

    up7 = Conv2DTranspose(filters[2], (2, 2), strides=(2, 2), name='up7', padding='same')(conv6)
    conv7 = concatenate([up7, conv3], name='merge3', axis=axis)
    conv7 = common_unit(conv7, stage='7', filters=filters[2])

    up8 = Conv2DTranspose(filters[1], (2, 2), strides=(2, 2), name='up8', padding='same')(conv7)
    conv8 = concatenate([up8, conv2], name='merge2', axis=axis)
    conv8 = common_unit(conv8, stage='8', filters=filters[1])

    up9 = Conv2DTranspose(filters[0], (2, 2), strides=(2, 2), name='up9', padding='same')(conv8)
    conv9 = concatenate([up9, conv1], name='merge1', axis=axis)
    conv9 = common_unit(conv9, stage='9', filters=filters[0])

    unet_output = Conv2D(
        num_class, (1, 1),
        activation='softmax',
        name='prediction',
        kernel_initializer='he_normal',
        padding='same',
        kernel_regularizer=l2(1e-4))(conv9)

    model = Model(inputs=img_input, outputs=unet_output)

    return model
