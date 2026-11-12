
#!/usr/bin/env python

from __future__ import print_function

from sys import path
from os.path import dirname as dir
import numpy as np
from base_controllers.utils.math_tools import Math


utils = Math()
actual_feetW = [np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1))]
stance_legs = [True, True,True,True]

actual_feetW[0] = np.array([0.3, 0.2, 0.0])
actual_feetW[1] = np.array([0.3, -0.2, 0.0])
actual_feetW[2] = np.array([-0.3, 0.2, 0.0])
actual_feetW[3] = np.array([-0.3, -0.2, 0.0])

stance_legs[0] = True
stance_legs[1] = True
stance_legs[2] = True
stance_legs[3] = False
point_to_test = np.array([0.25, 0.0, 0.0])


stance_feetW = []
for leg in range(4):
    if (stance_legs[leg]):
        stance_feetW.append(actual_feetW[leg])

print("stance feet : ",stance_feetW)
margin = utils.margin_from_poly(point_to_test,stance_legs, actual_feetW)
print("margin: ", margin)

A, b = utils.compute_half_plane_description(stance_feetW)
print("A: \n", A)
print("b: \n", b)

pt1 = np.array([1.0, 0.0, 0.0])

pt0 = np.array([0.1, 0.0, 0.0])
ret =  utils.compute_line_coeff( pt0, pt1)
print(ret.p, ret.q, ret.r)