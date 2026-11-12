# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""
import numpy as np

class AdmittanceControl():
    
    def __init__(self, ikin, Kx, Dx, conf):
        self.ikin = ikin
        self.conf = conf

        self.dp = np.zeros(3)
        self.dp_1_old = np.zeros(3)
        self.dp_2_old = np.zeros(3)
        self.Kx = Kx
        self.Dx = Dx
        self.q_postural =   self.conf['q_0']

    def setAdmittanceGains(self, Kx, Dx):
        self.Kx = Kx
        self.Dx = Dx

    def setPosturalTask(self, q_postural):
        self.q_postural = q_postural

    def computeAdmittanceReference(self, Fext, p_des, q_guess):
        # only Kx, Dx
        self.dp = np.linalg.inv(self.Kx + 1/self.conf['dt'] * self.Dx).dot(Fext + 1/self.conf['dt']*self.Dx.dot(self.dp_1_old))
        #only Kx (unstable)
        #self.dp = np.linalg.inv(self.Kx).dot(Fext)
        self.dp_2_old = self.dp_1_old
        self.dp_1_old = self.dp
        # you need to remember to remove the base offset! because pinocchio us unawre of that!
        q_des, ik_success, out_of_workspace = self.ikin.endeffectorInverseKinematicsLineSearch(p_des   + self.dp,
                                                                                               self.conf['ee_frame'],
                                                                                               q_guess, False, False,
                                                                                               postural_task=True,
                                                                                               w_postural=0.00001,
                                                                                               q_postural= self.q_postural)

        return q_des, p_des + self.dp

