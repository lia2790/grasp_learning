#!/usr/bin/env python

import unittest
import sys, os, glob

abspath = os.path.dirname(os.path.abspath(__file__))
sys.path.append(abspath+'/../')
from i16mc import make_moving_base_robot
from grasp_boxes_batch import getObjectPhalanxMeanContactPoint, countContactPoints, make_box
import numpy as np
from klampt.sim import WorldModel, SimpleSimulator
from klampt.math import vectorops, so3, se3
import plugins.soft_hand
from moving_base_control import set_moving_base_xform
#from klampt import vis
import time

class grasp_boxes_batchTest(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_getObjectPhalanxMeanContactPoint(self):
        world = WorldModel()
        world.loadElement(abspath+'/../data/terrains/plane.env')
        obj = make_box(world, 0.03, 0.05, 0.1)
        R,t = obj.getTransform()
        obj.setTransform(R, [0, 0, 0.1/2.])

        p = [0.015, -0.025, 0.1]
        q = [3.7494e-33, 6.12323e-17, 1.0, -6.12323e-17]
        o_T_p = np.array(se3.homogeneous((so3.from_quaternion(q), p)))
        w_T_o = np.array(se3.homogeneous(obj.getTransform()))
        p = (-0.03, -0.12, -0.013 + 0.02) # -0.013 + 0.02, from working exp at commit 98305499
        R = (0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        p_T_h = np.array(se3.homogeneous((R, p)))
        w_T_h_des = w_T_o.dot(o_T_p).dot(p_T_h)
        w_T_h_des_se3 = se3.from_homogeneous(w_T_h_des)

        robot = make_moving_base_robot('soft_hand', world)
        set_moving_base_xform(robot, w_T_h_des_se3[0], w_T_h_des_se3[1])
        sim = SimpleSimulator(world)

        # setup the preshrink
        visPreshrink = False  # turn this to true if you want to see the "shrunken" models used for collision detection
        for l in range(robot.numLinks()):
            sim.body(robot.link(l)).setCollisionPreshrink(visPreshrink)
        for l in range(world.numRigidObjects()):
            sim.body(world.rigidObject(l)).setCollisionPreshrink(visPreshrink)

        hand = plugins.soft_hand.HandEmulator(sim, 0, 6, 6)
        sim.addEmulator(0, hand)

        links_to_check = np.array([3, 4, 6, 8, 10, 13, 15, 17, 20, 22, 24, 27, 29, 31, 33, 35, 37]) + 6
        #vis.add("world", world)
        #vis.show()
        for i in range(100):
            #vis.lock()
            hand.setCommand([np.min((i/10.,1.0))])
            sim.simulate(0.01)
            sim.updateWorld()
            #vis.unlock()

            c_p, c_f = getObjectPhalanxMeanContactPoint(sim, obj, robot, links_to_check)
            c_p_all, c_f_all = getObjectPhalanxMeanContactPoint(sim, obj, robot)
            if i == 0:
                self.assertEqual(countContactPoints(c_p), 0)
                self.assertEqual(countContactPoints(c_p_all), 0)
            elif i == 99:
                self.assertTrue(countContactPoints(c_p) > 0)
                self.assertTrue(countContactPoints(c_p_all) > 0)
        #while vis.shown():
        #    pass

if __name__ == '__main__':
    unittest.main()