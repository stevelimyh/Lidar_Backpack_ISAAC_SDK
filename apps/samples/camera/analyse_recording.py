'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from engine.pyalice import Cask
import numpy
import time
import matplotlib.pyplot as plt
import argparse


def channel_dts(channel):
    '''Computes time difference of consecutive messages (in seconds)'''
    dts = []
    for i in range(len(channel) - 1):
        dt_ns = channel[i + 1].pubtime - channel[i].pubtime
        dts.append(dt_ns / 1000000.0)
    return dts


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Time difference historgram for cask channel')
    parser.add_argument('--root', dest='root',
                        help='The directory in which log files will be stored')
    parser.add_argument('--channel', dest='channel', default='color',
                        help='The channel for which to show time differences')
    args = parser.parse_args()

    cask = Cask(args.root)

    dts = channel_dts(cask[args.channel])

    print(numpy.histogram(dts))

    plt.title(args.channel)
    plt.hist(dts, bins='auto')
    plt.show()
