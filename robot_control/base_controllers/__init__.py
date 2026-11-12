#important to print properly matrix
import sys
import numpy as np
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
#do not create .pyc files
sys.dont_write_bytecode = True
# for new versions of pycharm
import matplotlib
matplotlib.use('TkAgg')