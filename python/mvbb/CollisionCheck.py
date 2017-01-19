#!/usr/bin/env python
from klampt import *
from klampt.vis.glrobotprogram import * #Per il simulatore
from klampt.model import collide
from moving_base_control import set_moving_base_xform, get_moving_base_xform
import math
from IPython import embed



def CheckCollision(world, robot, obj, collision = None):
    """
    :param world: world object
    :param robot: robot object
    :param obj: object to check collision against
    :collision: an optional WorldCollider
    :return: True if in collision
    """
    if collision is None:
        collision = collide.WorldCollider(world) #init
    r_o_coll = collision.robotObjectCollisions(robot,obj) #check collision robot-object. the output is generator
    list_r_o_coll = [] # make a list to know how many collisions we have been
    for coll in r_o_coll:
        list_r_o_coll.append(coll)
    r_w_coll = collision.robotTerrainCollisions(robot) # check collision robot-plane
    list_r_w_coll = [] # same as above. The output is generator so make a list to know how many collision we have been
    for coll in r_w_coll:
        list_r_w_coll.append(coll)
    if len(list_r_o_coll) > 0 or len(list_r_w_coll) >0: #if the length is greater that zero, we have collision
        return True
    else:
        return False

def CollisionTestInterpolate(world, robot, obj, w_P_h_goal, w_P_h_start = None):
    q_old = robot.getConfig()

    if not isinstance(w_P_h_goal, tuple):
        w_T_h_goal = se3.from_homogeneous(w_P_h_goal) #o_P_h is end-effector in object frame
    else:
        w_T_h_goal = w_P_h_goal

    if w_P_h_start is None:
        w_T_h_start = get_moving_base_xform(robot)
    else:
        if not isinstance(w_P_h_start, tuple):
            w_T_h_start = se3.from_homogeneous(w_P_h_start)  # o_P_h is end-effector in object frame
        else:
            w_T_h_start = w_P_h_start

    t_des = w_T_h_goal[1]
    t_curr = w_T_h_start[1]

    set_moving_base_xform(robot, w_T_h_goal[0], w_T_h_start[1])

    step_size = 0.01
    d = vectorops.distance(t_curr, t_des)

    n_steps = int(math.ceil(d / step_size))
    if n_steps == 0:    # if o_T_h_current == o_T_h_desired
        return CheckCollision(world, robot, obj)

    collider = collide.WorldCollider(world)

    for i in range(n_steps):
        t_int = vectorops.interpolate(t_curr,t_des,float(i+1)/n_steps)

        set_moving_base_xform(robot, w_T_h_goal[0], t_int)
        if CheckCollision(world, robot, obj, collider):
            robot.setConfig(q_old)
            return True

    robot.setConfig(q_old)
    return False


def CollisionTestPose(world, robot, obj, w_P_h):
    q_old = robot.getConfig()
    if not isinstance(w_P_h, tuple):
        w_T_h = se3.from_homogeneous(w_P_h) #o_P_h is end-effector in object frame
    else:
        w_T_h = w_P_h

    set_moving_base_xform(robot, w_T_h[0], w_T_h[1])

    coll = CheckCollision(world, robot, obj)

    robot.setConfig(q_old)
    return coll

def WillCollideDuringClosure(hand, obj):
    world = hand.world
    robot = hand.robot
    q_old = robot.getConfig()
    sigma_old = hand.getCommand()
    collision = collide.WorldCollider(world) #init
    list_r_w_coll = []  # same as above. The output is generator so make a list to know how many collision we have been
    for i in range(6):
        hand.setCommand([i/5.0])
        tau, q = hand.output(kinematic = True)
        robot.setConfig(q)
        r_w_coll = collision.robotTerrainCollisions(robot) # check collision robot-plane
        for coll in r_w_coll:
            list_r_w_coll.append(coll)
    hand.setCommand(sigma_old)
    robot.setConfig(q_old)
    if len(list_r_w_coll) >0: #if the length is greater that zero, we have collision
        return True
    else:
        return False
