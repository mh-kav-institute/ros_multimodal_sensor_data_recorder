#!/usr/bin/env python3

"""
***************************************
*                                     *
* Copyright (c) 2023                  *
*                                     *
* Technische Hochschule Aschaffenburg *
* Manuel Hetzel                       *
* KAV Labor                           *
* 63743 Aschaffenburg                 *
* Wuerzburger Str. 45                 *
*                                     *
***************************************

/**
 * @file utils.py for Xung data recording
 * @author Manuel Hetzel
 * @date 4.10.2023
 * @version 1.0
 *
 * @brief Provides lidar destaggering and command-line option helpers.
 *
*/
"""


import argparse
import numpy as np
from ouster.client import _client
from typing import List


os1_pixel_shift_by_row = [ 
    24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 
    24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 
    24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 
    24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 
    24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 
    24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 
    24, 16, 8, 0, 24, 16, 8, 0 ]


def _destagger(field: np.ndarray, shifts: List[int],
               inverse: bool) -> np.ndarray:
    """Perform the destagger operation.
    
    Args:
        field (object): Input field value.
        shifts (object): Input shifts value.
        inverse (object): Input inverse value.
    
    Returns:
        object: Result produced by the operation.
    """
    return {
        np.dtype(np.int8): _client.destagger_int8,
        np.dtype(np.int16): _client.destagger_int16,
        np.dtype(np.int32): _client.destagger_int32,
        np.dtype(np.int64): _client.destagger_int64,
        np.dtype(np.uint8): _client.destagger_uint8,
        np.dtype(np.uint16): _client.destagger_uint16,
        np.dtype(np.uint32): _client.destagger_uint32,
        np.dtype(np.uint64): _client.destagger_uint64,
        np.dtype(np.single): _client.destagger_float,
        np.dtype(np.double): _client.destagger_double,
    }[field.dtype](field, shifts, inverse)


def destagger(fields: np.ndarray, pixels_per_column=128, columns_per_frame=1024, pixel_shift_by_row=[], inverse=False) -> np.ndarray:
    """Return a destaggered copy of the provided fields.

    In the default staggered representation, each column corresponds to a
    single timestamp. A destaggered representation compensates for the
    azimuth offset of each beam, returning columns that correspond to a
    single azimuth angle.

    Args:
        info: Sensor metadata associated with the provided data
        fields: A numpy array of shape H X W or H X W X N
        inverse: perform inverse "staggering" operation

    Returns:
        A destaggered numpy array of the same shape
    """
    h = pixels_per_column
    w = columns_per_frame
    shifts = pixel_shift_by_row

    # remember original shape
    shape = fields.shape
    fields = fields.reshape((h, w, -1))

    # apply destagger to each channel
    # note: astype() needed due to some strange behavior of the pybind11
    # bindings. The wrong overload is chosen otherwise (due to the indexing?)
    return np.dstack([_destagger(fields[:, :, i], shifts, inverse) for i in range(fields.shape[2])]).reshape(shape)


class BaseOptions():
    """parse command line arguments"""

    def __init__(self):
        """init parser"""
        self.parser = argparse.ArgumentParser(description='Xung alb record node')
        self.initialized = False

    def initialize(self, parser):
        """define available input arguments"""

        parser.add_argument('-dir', type=str, default='/workspace/data', help='root directory for recording data')
        parser.add_argument('-albs', type=list, default=['fusion', 'edge_m1', 'edge_m3'], help='name of target albs to connect to (edge_m1, edge_m3, fusion')
        parser.add_argument('-topics', type=str, default=['augmented_cloud', 'tracked_objects'], help='name of target alb topics to record')
        parser.add_argument('-cpuid', type=int, default=0, help='ID of first target cpu core to use (one needed), e.g -cpuid=0 for cpu core 0')

        return parser

    def gather_options(self):
        """add additional model-specific options"""

        if not self.initialized:
            parser = self.initialize(self.parser)

        # Get argument values
        args, unknown = parser.parse_known_args()
        #opt = parser.parse_args()
        return args

    def parse(self):
        """parse the options"""

        opt = self.gather_options()
        self.print_options(opt)
        ret = self.check_args(opt)
        result = [ret, opt]
        return result

    @staticmethod
    def check_args(opt):
        """check if all necessary input arguments have been defined by user"""

        ret = True

        for i in vars(opt):
            if getattr(opt, i) is None:
                print('--- Error: Missing input argument: %s' % (str(i)))
                ret = False

        return ret

    @staticmethod
    def print_options(opt):
        """print and save options"""

        print('-----------------')
        print('Input Arguments: ')
        for k, v in sorted(vars(opt).items()):
            print('\t%s: %s' % (str(k), str(v)))
        print('-----------------')