from __future__ import print_function
from roslib.scriptutil import get_param_server
from roslib.names import ns_join, get_ros_namespace, make_caller_id
import socket

import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import rospy as ros
from rospy.exceptions import ROSException
import copy


class Utils:

    def __init__(self):
        
        
        self.leg_map = {
            "LF": 0,
            "LH": 1,
            "RF": 2,
            "RH": 3
        }
        self.crd = {
            "X": 0,
            "Y": 1,
            "Z": 2,
        }


        self.sp_crd = {
            "LX": 0,
            "LY": 1,
            "LZ": 2,
            "AX": 3,
            "AY": 4,
            "AZ": 5,
        }



    def getSegment(self, var, index, size):
        return var[index:index+size]

    def linPart(self, var):
        index = self.sp_crd["LX"]
        return var[index:index+3]

    def angPart(self, var):
        index = self.sp_crd["AX"]
        return var[index:index+3]

    ########################################################################

    #manage param server

    ########################################################################



    def succeed(self,args):
        """
        Utility routine for checking ROS XMLRPC API call
        @return: value field from ROS xmlrpc call
        @rtype: XmlRpcLegalValue
        @raise ROSParamException: if call to ROS xmlrpc API did not succeed
        """
        code, msg, val = args
        if code != 1:
            raise ros.ROSException(msg)
        return val

    def get_caller_id(self):
        """
        @return: caller ID for rosparam ROS client calls
        @rtype: str
        """
        return make_caller_id('rosparam-%s' % os.getpid())

    def putIntoParamServer(self, data):

        try:
            self.succeed(get_param_server().setParam(self.get_caller_id(),  "", data))

        except socket.error:
            raise ROSException("Unable to communicate with master!")
        print ("set parameter [%s] to [%s]" % ('hyq', data))
        eval(help( get_param_server().setParam()) )								
        pass

    def putIntoGlobalParamServer(self,label, data, verbose = False):

        try:
            ros.set_param(label, data)
        except socket.error:
            raise ROSException("Unable to communicate with master!")
        if (verbose):
            print ("set parameter %s into global param server" % label)
        pass

 
#########################################################################

    def getIdx(self, leg, coord, numberOfJointsPerLeg = 3):
        return self.leg_map[leg]*numberOfJointsPerLeg + self.crd[coord]

    def setLegJointState(self, legid,  input, jointState, numberOfJointsPerLeg = 3):
        if isinstance(legid, str):
            jointState[self.leg_map[legid]*numberOfJointsPerLeg:self.leg_map[legid]*numberOfJointsPerLeg+numberOfJointsPerLeg] = input
        elif isinstance(legid, int):
            jointState[legid*numberOfJointsPerLeg:(legid+1)*numberOfJointsPerLeg] = input

    def getLegJointState(self, legid,  jointState, numberOfJointsPerLeg = 3):
        if isinstance(legid, str):
            return jointState[self.leg_map[legid]*numberOfJointsPerLeg:(self.leg_map[legid]+1)*numberOfJointsPerLeg]
        elif isinstance(legid, int):
            return jointState[legid * numberOfJointsPerLeg:(legid+1) * numberOfJointsPerLeg]

    def spy(self, var):
        plt.spy(var)
        plt.show()


    def detectLiftOff(self, swing,idx, leg):
        if ((swing[leg, idx-1] == 0) and (swing[leg, idx] == 1)):

            return True
        else:

            return False

    def detectTouchDown(self, swing,idx, leg):
        if ((swing[leg, idx] == 1) and (swing[leg, idx+1] == 0)):

            return True
        else:

            return False

    def detectHapticTouchDown(self, grForcesW, leg, force_th):
        grfleg = self.getLegJointState(leg, grForcesW)
        if grfleg[2]>=force_th:
            return True
        else:
            return False

    def mapFromRos(self, ros_in):
        # ros_out = np.zeros_like(ros_in)
        # ros_out[0:3] = ros_in[0:3]
        # ros_out[3:6] = ros_in[6:9]
        # ros_out[6:9] = ros_in[3:6]
        # ros_out[9:12] = ros_in[9:12]
        # if (len(ros_in)>12):
        #     ros_out[12:] = ros_in[12:] #copy last joints if any
        return ros_in
        
    
    def mapToRos(self, ros_in):
        # ros_out = np.zeros_like(ros_in)
        # ros_out[0:3] = ros_in[0:3]
        # ros_out[3:6] = ros_in[6:9]
        # ros_out[6:9] = ros_in[3:6]
        # ros_out[9:12] = ros_in[9:12]
        # if (len(ros_in)>12):
        #     ros_out[12:] = ros_in[12:] #copy last joints if any
        return ros_in
        
    def mapIndexToRos(self, index_in: object) -> object:
        # index_out =  index_in
        # if index_in == 1:
        #     index_out = 2
        # if index_in == 2:
        #     index_out = 1
        return index_in

    def mapLegListToRos(self, list: object) -> object:
        # tmp1 = list[1]
        # tmp2 = list[2]
        #
        # list[1] = tmp2
        # list[2] = tmp1

        return list

    def get_dict_keys(dict):
        names=list(dict.keys())
        names.sort()
        return  names


    def tic():
        # Homemade version of matlab tic and toc functions
        import time
        global startTime_for_tictoc
        startTime_for_tictoc = time.time()


    def toc():
        import time
        if 'startTime_for_tictoc' in globals():
            print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
        else:
            print("Toc: start time not set")

    def full_listOfArrays(self, length, rows, cols=0, value=np.nan):
        # create a list of length independent np.ndarrays of shape (rows, cols) with all the entries set to value
        # e.g. full_array_list(2, 1, 2, 5) returns
        # [ array([[5], [5]]), array([[5], [5]])]
        if cols == 0:
            a = np.full(rows, value)
        else:
            a = np.full((rows, cols), value)
        return self.listOfArrays(length, a)


    def listOfArrays(self, length, array):
        # create a list of length independent np.ndarrays
        L = []
        for i in range(length):
            L.append(copy.deepcopy(array))
        return L

            