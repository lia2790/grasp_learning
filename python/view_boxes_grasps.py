#!/usr/bin/env python

import pkg_resources
pkg_resources.require("klampt>=0.7.0")
from klampt import *
from klampt import vis
from klampt.vis.glrobotprogram import *
from klampt.math import *
from klampt.model import collide
from klampt.io import resource
from klampt.sim import *
from moving_base_control import *
import csv
import importlib
import math
import os
import string
import sys
import time
import pickle

from klampt.math import so3, se3
import numpy as np
from IPython import embed
from mvbb.draw_bbox import draw_GL_frame, draw_bbox
from i16mc import make_moving_base_robot
from mvbb.CollisionCheck import CheckCollision, CollisionTestInterpolate, CollisionTestPose
from mvbb.box_db import MVBBLoader
from grasp_boxes_batch import make_box

robot_files = {
    'soft_hand':'data/robots/soft_hand.urdf'
}

class GraspVisualizer(GLNavigationProgram):
    def __init__(self, world, p_T_h, box_db, module):
        GLNavigationProgram.__init__(self, 'XForm Visualizer')
        self.robot = None
        self.world = world
        self.box_db = box_db
        self.keys = box_db.db_simulated.keys()
        self.module = module

        self.box_dims = None
        self.pose = None
        self.box_dims_i = 0
        self.pose_i = None

        self.sim = None
        self.hand = None

        self.robot = world.robot(0)
        self.p_T_h = p_T_h

        self.sim = SimpleSimulator(self.world)
        self.hand = self.module.HandEmulator(self.sim, 0, 6, 6)
        self.sim.addEmulator(0, self.hand)
        # the next line latches the current configuration in the PID controller...
        self.sim.controller(0).setPIDCommand(self.robot.getConfig(),
                                             vectorops.mul(self.robot.getVelocity(), 0.0))



    def show_next_grasp(self):
        poses = self.box_db.db_simulated[self.keys[self.box_dims_i]]
        if self.pose_i is None:
            pose_i = 0
        else:
            pose_i = self.pose_i + 1
            if pose_i >= len(poses):
                print "No valid poses found for box", self.keys[self.box_dims_i]
                self.box_dims_i = self.box_dims_i + 1
                if self.box_dims_i >= len(self.keys):
                    print "Scanned all poses for all boxes, wrapping around"
                    self.box_dims_i = 0
                    self.pose_i = None
                    return False
                else:
                    print "Switched to next box"
                    poses = self.box_db.db_simulated[self.keys[self.box_dims_i]]
                    pose_i = 0
        while math.isnan(poses[pose_i]['q'][0]):
            pose_i = pose_i + 1
            if pose_i >= len(poses):
                print "No valid poses found for box", self.keys[self.box_dims_i]
                self.box_dims_i = self.box_dims_i + 1
                if self.box_dims_i >= len(self.keys):
                    print "Scanned all poses for all boxes, wrapping around"
                    self.box_dims_i = 0
                    self.pose_i = None
                    return False
                else:
                    print "Switched to next box"
                    poses = self.box_db.db_simulated[self.keys[self.box_dims_i]]
                    pose_i = 0
        print "Visualizing pose", pose_i, "for box", self.keys[self.box_dims_i], ":", poses[pose_i]

        self.pose_i = pose_i
        if self.world.numRigidObjects() > 0:
            self.world.remove(self.world.rigidObject(0))
        self.box_dims = self.keys[self.box_dims_i]
        self.pose = self.box_db.db_simulated[self.box_dims][self.pose_i]
        obj = make_box(self.world,
                       self.box_dims[0],
                       self.box_dims[1],
                       self.box_dims[2])
        self.hand.setConfiguration(self.pose['q'])
        o_T_p = self.pose['T']
        w_T_o = np.array(se3.homogeneous(obj.getTransform()))
        w_T_h_des = w_T_o.dot(o_T_p).dot(self.p_T_h)
        w_T_h_des_se3 = se3.from_homogeneous(w_T_h_des)
        set_moving_base_xform(self.robot, w_T_h_des_se3[0], w_T_h_des_se3[1])
        h_T_o = self.pose['h_T_o']
        w_T_o_curr = w_T_h_des.dot(h_T_o)
        w_T_o_curr_se3 = se3.from_homogeneous(w_T_o_curr)
        obj.setTransform(*w_T_o_curr_se3)

    def display(self):
        if self.pose is not None:
            self.world.drawGL()

    def keyboardfunc(self, c, x, y):
        # Put your keyboard handler here
        # the current example toggles simulation / movie mode
        print c, "pressed"

        if c == 'n':
            self.show_next_grasp()

        self.refresh()

    def idle(self):
        pass

def launch_grasp_viewer(robotname, box_db):
    """
    Launches a very simple program that spawns a hand, and object and grasp points to check everything
    has been correctly saved.
    """

    world = WorldModel()
    robot = make_moving_base_robot(robotname, world)
    set_moving_base_xform(robot, se3.identity()[0], se3.identity()[1])

    xform = resource.get("default_initial_%s.xform"%robotname,description="Initial hand transform",default=se3.identity(),world=world,doedit=False)
    print "Transform:", xform
    p_T_h = np.array(se3.homogeneous(xform))

    module = importlib.import_module('plugins.' + robotname)
    program = GraspVisualizer(world, p_T_h, box_db, module)
    vis.setPlugin(program)
    program.reshape(800, 600)

    vis.show()

    # this code manually updates the visualization

    while vis.shown():
        time.sleep(0.01)

    vis.kill()
    return

if __name__ == '__main__':
    box_db = None
    try:
        import os.path
        filename = os.path.splitext(sys.argv[1])[0]
    except:
        filename = 'box_db'

    box_db = MVBBLoader(filename, 19, 16)

    poses = box_db.db_simulated
    suc_pose_count = 0
    pose_count = 0
    for i in poses.keys():
        pose_count += len(poses[i])
        for pose in poses[i]:
            if not math.isnan(pose['q'][0]):
                suc_pose_count += 1
    print "Found", suc_pose_count, "successful grasps out of", pose_count, "grasps  "

    launch_grasp_viewer("soft_hand", box_db)