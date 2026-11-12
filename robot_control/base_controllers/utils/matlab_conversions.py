# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 09:43:27 2018

@author: romeo orsolino
"""
#to be compatible with python3
from __future__ import print_function
import numpy as np


def mat_matrix2python(input):
    np_mat = np.zeros((input.size[0], input.size[1]))
    for i in range(input.size[0]):
        np_mat[i,:] = input[i][:]
    return np_mat

def mat_vector2python(input):
    arr_size = max(input.size[0], input.size[1])
    np_arr = np.zeros((arr_size)) # force to row
    for i in range(arr_size):
        if input.size[0] > input.size[1]:
            np_arr[i] = (input.reshape(input.size[1], input.size[0]))[0][i]
        else:
            np_arr[i] = input[0][i]
    return np_arr




