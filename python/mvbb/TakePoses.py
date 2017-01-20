#!/usr/bin/env python

from klampt import *
from klampt.math import so3, se3
import numpy as np


def SimulationPoses(o_T_h_des, w_T_h, w_T_o):
    #w_T_h end-effector in word frame
    #h_T_o object in end-effector frame
    #w_T_o pbject in word frame

    if not isinstance(o_T_h_des, tuple):
        o_T_h_des = se3.from_homogeneous(o_T_h_des)

    if not isinstance(w_T_h, tuple):
        w_T_h = se3.from_homogeneous(w_T_h)

    if not isinstance(w_T_o, tuple):
        w_T_o = se3.from_homogeneous(w_T_o)

    posedict = {}
    posedict['desired'] = [se3.inv(o_T_h_des)]
    posedict['actual'] = [se3.mul(se3.inv(w_T_h), w_T_o)]
    return posedict
