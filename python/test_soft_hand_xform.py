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
import importlib
import math
import os
import string
import sys
import time

from klampt.math import so3, se3
import pydany_bb
import numpy as np
from IPython import embed
from mvbb.draw_bbox import draw_GL_frame, draw_bbox
from i16mc import make_object, make_moving_base_robot

from klampt.io import resource

robot_files = {
    'soft_hand':'data/robots/soft_hand.urdf'
}
class XFormVisualizer(GLNavigationProgram):
    def __init__(self, world, xform):
        GLNavigationProgram.__init__(self, 'XForm Visualizer')
        self.robot = None
        self.world = world
        if world.numRobots > 0:
            self.robot = world.robot(0)
            set_moving_base_xform(self.robot, xform[0], xform[1])

    def display(self):
        if self.robot is not None:
            self.robot.drawGL()
            T = se3.identity()
            draw_GL_frame(T)


    def keyboardfunc(self, c, x, y):
        # Put your keyboard handler here
        # the current example toggles simulation / movie mode
        print c, "pressed"

        if c == 's':
            pass

        self.refresh()

    def idle(self):
        pass

def launch_xform_viewer(robotname):
    """Launches a very simple program that spawns an object from one of the
    databases.
    It launches a visualization of the mvbb decomposition of the object, and corresponding generated poses.
    It then spawns a hand and tries all different poses to check for collision
    """

    world = WorldModel()
    robot = make_moving_base_robot(robotname, world)
    set_moving_base_xform(robot, se3.identity()[0], se3.identity()[1])

    xform = resource.get("default_initial_%s.xform"%robotname,description="Initial hand transform",default=se3.identity(),world=world,doedit=False)
    print "Transform:", xform
    program = XFormVisualizer(world, xform)
    vis.setPlugin(program)
    program.reshape(800, 600)

    vis.show()

    # this code manually updates the visualization

    while vis.shown():
        time.sleep(0.01)

    vis.kill()
    return

if __name__ == '__main__':

    launch_xform_viewer("soft_hand")