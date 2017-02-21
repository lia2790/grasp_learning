#!/usr/bin/env python

import math
from klampt.math import so3, se3, vectorops
from klampt import vis
from klampt.vis.glprogram import *
import numpy as np


def draw_GL_frame(T, axis_length=0.1, color = None):
    """Draws the rigid transform T as a set of axis-oriented lines
    of length axis_length."""
    R,t = T
    if len(R)!=9 or len(t)!=3:
        print "Incorrect sizes",len(R),len(t)
        raise RuntimeError("")
    #draw transform widget

    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    glLineWidth(1.5)
    if color is None:
        glColor3f(1, 0, 0)
    else:
        glColor3f(*color)
    glBegin(GL_LINES)
    glVertex3fv(t)
    glVertex3fv(vectorops.madd(t, R[0:3], axis_length))
    glEnd()
    if color is None:
        glColor3f(0, 1, 0)
    glBegin(GL_LINES)
    glVertex3fv(t)
    glVertex3fv(vectorops.madd(t, R[3:6], axis_length))
    glEnd()
    if color is None:
        glColor3f(0, 0, 1)
    glBegin(GL_LINES)
    glVertex3fv(t)
    glVertex3fv(vectorops.madd(t, R[6:9], axis_length))
    glEnd()
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)

    """
    glBegin(GL_LINES)
    glColor3f(1,1,1)
    glVertex3fv(t)
    glColor3f(1,0,0)
    glVertex3fv(vectorops.madd(t,R[0:3],axis_length))
    glColor3f(1,1,1)
    glVertex3fv(t)
    glColor3f(0,1,0)
    glVertex3fv(vectorops.madd(t,R[3:6],axis_length))
    glColor3f(1,1,1)
    glVertex3fv(t)
    glColor3f(0,0,1)
    glVertex3fv(vectorops.madd(t,R[6:9],axis_length))
    glColor3f(0,0,0)
    glEnd()
    """

def draw_bbox(isobox, T):
    x_0 = isobox[0,:]
    x_1 = isobox[1,:]
    vertices = np.zeros((8,3))
    vertices[0,:] = np.array([x_0[0], x_0[1], x_0[2]])
    vertices[1,:] = np.array([x_0[0], x_1[1], x_0[2]])
    vertices[2,:] = np.array([x_1[0], x_1[1], x_0[2]])
    vertices[3,:] = np.array([x_1[0], x_0[1], x_0[2]])
    vertices[4,:] = np.array([x_0[0], x_0[1], x_1[2]])
    vertices[5,:] = np.array([x_0[0], x_1[1], x_1[2]])
    vertices[6,:] = np.array([x_1[0], x_1[1], x_1[2]])
    vertices[7,:] = np.array([x_1[0], x_0[1], x_1[2]])

    w_R_l = T[0:3, 0:3]
    w_t_l = T[0:3, 3]
    w_vertices = w_R_l.dot(vertices.T).T + w_t_l

    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    glLineWidth(3.0)
    glColor3f(1, 0, 1)
    glBegin(GL_LINES)
    glVertex3f(*w_vertices[0,:])
    glVertex3f(*w_vertices[1,:])
    glVertex3f(*w_vertices[1,:])
    glVertex3f(*w_vertices[2,:])
    glVertex3f(*w_vertices[2,:])
    glVertex3f(*w_vertices[3,:])
    glVertex3f(*w_vertices[3,:])
    glVertex3f(*w_vertices[0,:])
    glVertex3f(*w_vertices[4,:])
    glVertex3f(*w_vertices[5,:])
    glVertex3f(*w_vertices[5,:])
    glVertex3f(*w_vertices[6,:])
    glVertex3f(*w_vertices[6,:])
    glVertex3f(*w_vertices[7,:])
    glVertex3f(*w_vertices[7,:])
    glVertex3f(*w_vertices[4,:])
    glVertex3f(*w_vertices[0,:])
    glVertex3f(*w_vertices[4,:])
    glVertex3f(*w_vertices[1,:])
    glVertex3f(*w_vertices[5,:])
    glVertex3f(*w_vertices[2,:])
    glVertex3f(*w_vertices[6,:])
    glVertex3f(*w_vertices[3,:])
    glVertex3f(*w_vertices[7,:])
    glEnd()
    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
