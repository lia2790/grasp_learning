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
from mvbb.graspvariation import PoseVariation
from mvbb.TakePoses import SimulationPoses
from mvbb.draw_bbox import draw_GL_frame, draw_bbox
from i16mc import make_moving_base_robot
from mvbb.CollisionCheck import CheckCollision, CollisionTestInterpolate, CollisionTestPose
from mvbb.box_db import MVBBLoader


objects = {}
objects['ycb'] = [f for f in os.listdir('data/objects/ycb')]
objects['apc2015'] = [f for f in os.listdir('data/objects/apc2015')]
robots = ['reflex_col', 'soft_hand', 'reflex']

def make_box(world, x_dim, y_dim, z_dim, mass=0.1):
    """Makes a new axis-aligned box centered at the origin with
    dimensions width x depth x height. The box is a RigidObject
    with automatically determined inertia.
        """
    boxgeom = Geometry3D()
    boxgeom.loadFile("data/objects/cube.tri")
    #boxgeom.transform([x_dim, 0, 0, 0, y_dim, 0, 0, 0, z_dim], [-x_dim * 0.5, -y_dim * 0.5, -z_dim * 0.5])
    # box is centered at the origin
    boxgeom.transform([x_dim, 0, 0, 0, y_dim, 0, 0, 0, z_dim], [0., 0., 0.])

    print "Making a box a rigid object"
    bmass = Mass()
    bmass.setMass(mass)
    bmass.setCom([0, 0, 0])
    bmass.setInertia([x_dim / 12, y_dim / 12, z_dim / 12])
    box = world.makeRigidObject("box")
    box.geometry().set(boxgeom)
    box.appearance().setColor(0.6, 0.3, 0.2, 1.0)
    box.setMass(bmass)
    cparams = box.getContactParameters()
    cparams.kStiffness = 100000
    cparams.kDamping = 30000
    cparams.kRestitution = 0.5

    return box

class FilteredMVBBTesterVisualizer(GLRealtimeProgram):
    def __init__(self, poses, world, p_T_h, R, module):
        GLRealtimeProgram.__init__(self, "FilteredMVBBTEsterVisualizer")
        self.world = world
        self.p_T_h = p_T_h
        self.h_T_p = np.linalg.inv(self.p_T_h)
        self.poses = []
        self.hand = None
        self.is_simulating = False
        self.curr_pose = None
        self.robot = self.world.robot(0)
        self.q_0 = self.robot.getConfig()
        self.w_T_o = None
        self.obj = None
        self.t_0 = None
        self.object_com_z_0 = None
        self.object_fell = None
        self.sim = None
        self.module = module
        self.running = True
        self.db = None
        
        if self.world.numRigidObjects() > 0:
            self.obj = self.world.rigidObject(0)
            self.w_T_o = np.array(se3.homogeneous(self.obj.getTransform()))
            for p in poses:
                if not self.db.has_simulation(self.obj.getName(), p):
                    self.poses.append(p)
                else:
                    print "Pose", p, "already simulated"
        else:
            "Warning: during initialization of visualizer object is still not loaded in world"
            selp.poses = poses

        print "Will simulate", len(self.poses), "poses,"

    def display(self):
        if self.running:
            self.world.drawGL()

            for pose in self.poses+self.poses_variations:
                T = se3.from_homogeneous(pose)
                draw_GL_frame(T, color=(0.5,0.5,0.5))
            if self.curr_pose is not None:
                T = se3.from_homogeneous(self.curr_pose)
                draw_GL_frame(T)

            hand_xform = get_moving_base_xform(self.robot)
            w_T_p = np.array(se3.homogeneous(hand_xform)).dot(self.h_T_p)
            w_T_p_se3 = se3.from_homogeneous(w_T_p)
            draw_GL_frame(w_T_p_se3)

    def idle(self):
        if not self.running:
            return

        if self.world.numRigidObjects() > 0:
            self.obj = self.world.rigidObject(0)
        elif self.obj is None:
            return

        if not self.is_simulating:
            if len(self.poses) > 0:
                self.curr_pose = self.poses.pop(0)

                print "Simulating Next Pose Grasp"
                print self.curr_pose
            else:
                print "Done testing all", len(self.poses), "poses for object", self.obj.getName()
                print "Quitting"
                self.running = False
                vis.show(hidden=True)
                return

            w_T_o_se3 = se3.from_homogeneous(self.w_T_o)
            self.obj.setTransform(w_T_o_se3[0], w_T_o_se3[1])
            w_T_h_des_se3 = se3.from_homogeneous(self.w_T_o.dot(self.curr_pose).dot(self.p_T_h))
            self.robot.setConfig(self.q_0)
            set_moving_base_xform(self.robot, w_T_h_des_se3[0], w_T_h_des_se3[1])

            if self.sim is None:
                self.sim = SimpleSimulator(self.world)
                self.hand = self.module.HandEmulator(self.sim, 0, 6, 6)
                self.sim.addEmulator(0, self.hand)
                # the next line latches the current configuration in the PID controller...
                self.sim.controller(0).setPIDCommand(self.robot.getConfig(), self.robot.getVelocity())

                obj_b = self.sim.body(self.obj)
                obj_b.setVelocity([0., 0., 0.],[0., 0., 0.])

                # setup the preshrink
                visPreshrink = False  # turn this to true if you want to see the "shrunken" models used for collision detection
                for l in range(self.robot.numLinks()):
                    self.sim.body(self.robot.link(l)).setCollisionPreshrink(visPreshrink)
                for l in range(self.world.numRigidObjects()):
                    self.sim.body(self.world.rigidObject(l)).setCollisionPreshrink(visPreshrink)
                self.db = MVBBLoader(self.hand.n_u + self.hand.n_d)

            self.object_com_z_0 = getObjectGlobalCom(self.obj)[2]
            self.object_fell = False
            self.t_0 = self.sim.getTime()
            self.is_simulating = True

        if self.is_simulating:
            t_lift = 1.3 # when to lift
            d_lift = 1.0 # duration
            # print "t:", self.sim.getTime() - self.t_0
            object_com_z = getObjectGlobalCom(self.obj)[2]
            w_T_h_curr_se3 = get_moving_base_xform(self.robot)
            w_T_h_des_se3 = se3.from_homogeneous(self.w_T_o.dot(self.curr_pose).dot(self.p_T_h))


            if self.sim.getTime() - self.t_0 == 0:
                # print "Closing hand"
                self.hand.setCommand([1.0])
            elif (self.sim.getTime() - self.t_0) >= t_lift and (self.sim.getTime() - self.t_0) <= t_lift+d_lift:
                # print "Lifting"
                t_i = w_T_h_des_se3[1]
                t_f = vectorops.add(t_i, (0,0,0.2))
                u = np.min((self.sim.getTime() - self.t_0 - t_lift, 1))
                send_moving_base_xform_PID(self.sim.controller(0), w_T_h_des_se3[0], vectorops.interpolate(t_i, t_f ,u))

            if (self.sim.getTime() - self.t_0) >= t_lift: # wait for a lift before checking if object fell
                d_hand = w_T_h_curr_se3[1][2] - w_T_h_des_se3[1][2]
                d_com = object_com_z - self.object_com_z_0
                if d_hand - d_com > 0.1:
                    self.object_fell = True
                    print "!!!!!!!!!!!!!!!!!!"
                    print "Object fell"
                    print "!!!!!!!!!!!!!!!!!!"

            self.sim.simulate(0.01)
            self.sim.updateWorld()

            if not vis.shown() or (self.sim.getTime() - self.t_0) >= 2.5 or self.object_fell:
                if vis.shown(): # simulation stopped because it was successful
                    print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
                    print "Saving grasp, object fall status:", "fallen" if self.object_fell else "grasped"
                    print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
                    w_T_h_curr = np.array(se3.homogeneous(w_T_h_curr_se3))
                    w_T_o_curr = np.array(se3.homogeneous(self.obj.getTransform()))
                    h_T_o = np.linalg.inv(w_T_h_curr).dot(w_T_o_curr)
                    q_grasp = self.hand.getConfiguration()

                    c_p, c_f = getObjectPhalanxMeanContactPoint(self.obj, self.robot)

                    self.db.save_simulation(getObjectDims(self.obj), self.curr_pose, h_T_o,
                                            q_grasp, c_p, c_f)
                self.is_simulating = False
                self.sim = None

def getObjectGlobalCom(obj):
    return se3.apply(obj.getTransform(), obj.getMass().getCom())

def getObjectDims(obj):
    # when margins are zero, the box dims are simply the bounding box dimensions
    BB = obj.geometry().getBB()
    dims = vectorops.sub(BB[1],BB[0])
    return tuple(dims)

def getObjectPhalanxMeanContactPoint(obj, robot):
    """
    Returns a contact point for each link in the robot, which is the simple arithmetic mean of all contact points,
    expressed in the object frame of reference
    This is not a contact centroid.
    Also returns the total force applied by the link to the object, and the total moment applied to the object
    (with reference pole in the object origin), expressed in the object frame of reference.

    :param obj: object to grasp
    :param robot: robot grasping the object
    :return: (cps_avg, wrench_avg) where cps_avg is a vector 3*n_links, and wrench_avg is a vector 6*n_links
    """
    oId = obj.getId()
    lIds = []
    lId_to_lIndex = {}
    for l_i in xrange(robot.numLinks()):
        link = robot.link(l_i)
        lIds.append(link.getID())
        lId_to_lIndex[link.getID()] = l_i

    cps_avg = np.array([float('nan')] * 3 * len(lIds))
    wrench_avg = np.array([float('nan')] * 6 * len(lIds))

    for lId in lIds:
        clist = self.sim.getContacts(oId, lId)
        for c in clist:
            pavg = vectorops.add(pavg, c[0:3])
            navg = vectorops.add(navg, c[3:6])

        if len(clist) > 0:
            pavg = vectorops.div(pavg, len(clist))
            navg = vectorops.div(navg, len(clist))
            l_i = lId_to_lIndex[lId]
            cps_avg[l_i*3:l_i*3+3] = pavg

            wrench_avg[l_i*3:l_i*3+3] = sim.contactForce(oId, i)
            wrench_avg[l_i*3+3:l_i*3+6] = sim.contactTorque(oId, i)
    return (cps_avg, wrench_avg)


def launch_test_mvbb_grasps(robotname, box_db):
    """Launches a very simple program that spawns a box with dimensions specified in boxes_db.
    """

    world = WorldModel()
    world.loadElement("data/terrains/plane.env")
    robot = make_moving_base_robot(robotname, world)
    xform = resource.get("default_initial_%s.xform" % robotname, description="Initial hand transform",
                         default=se3.identity(), world=world, doedit=False)
    for box_dims, box_params in box_db.db:
        obj = make_box(box_dims[0],
                       box_dims[1],
                       box_dims[2])

        poses = [p['T'] for p in box_params]
        poses_filtered = []

        R,t = obj.getTransform()
        obj.setTransform(R, ['', 0, z_dim/2.])

        # embed()

        p_T_h = np.array(se3.homogeneous(xform))

        for pose in poses:
            if CollisionTestPose(world, robot, obj, pose):
                print "Pose", pose, "has been filtered since it is in collision for box", box
            else:
                poses_filtered.append(pose)

        # embed()
        # create a hand emulator from the given robot name
        module = importlib.import_module('plugins.' + robotname)
        # emulator takes the robot index (0), start link index (6), and start driver index (6)

        program = FilteredMVBBTesterVisualizer(poses_filtered,
                                               world,
                                               p_T_h,
                                               module)

        vis.setPlugin(None)
        vis.setPlugin(program)
        program.reshape(800, 600)

        vis.show()
        # this code manually updates the visualization
        while vis.shown():
            time.sleep(0.1)
    return

if __name__ == '__main__':
    box_db = []
    try:
        import os.path
        filename = os.path.splitext(sys.argv[1])[0]
    except:
        filename = 'box_db'
    try:
        box_db = MVBBLoader(filename)
        launch_test_mvbb_grasps("soft_hand", box_db)
    except:
        print "Error loading", filename

