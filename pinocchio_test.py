#
# Copyright (c) 2016 CNRS
# Read Roy Featherstones book: @ Rigid Body Dynamics Algorithms @
# Tutorial: https://github.com/stack-of-tasks/pinocchio/blob/f665f3f93669860beb8e80388ed3b52043ddd868/doc/d-practical-exercises/4-dyn.md
# Jacobian Time Derivative: https://github.com/stack-of-tasks/pinocchio/issues/901
# Coriolis: https://github.com/stack-of-tasks/pinocchio/blob/db8fe0add7d10ea31ff13aa677e30fcc3b569e42/bindings/python/pinocchio/derivative/xm.py
# DART is another rigid body dynamics library worth looking into
#

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from dcrba import *
from numpy.linalg.linalg import norm
from sys import argv
from os.path import dirname, join, abspath
np.random.seed(0)

dir = '/home/ash/Ash/repo/ash_pybullet'

urdf_filename = dir + '/ur5_robot.urdf'
robot = RobotWrapper.BuildFromURDF(urdf_filename, [dir])
# robot = RobotWrapper.BuildFromURDF(urdf_filename, [dir], pin.JointModelFreeFlyer())
q = rand(robot.model.nq)
vq = rand(robot.model.nv)
aq = rand(robot.model.nv)
tau = rand(robot.model.nv)  # Motor torque
#
"""
M(q) ddq + C(q, dq) + G(q) = Tau
b = C(q, dq) + G(q) (bias term as in Featherstone)
"""
# compute dynamic drift -- Coriolis, centrifugal, gravity
b = pin.rnea(robot.model, robot.data, q, vq, np.zeros_like(aq))
# compute mass matrix M
M = pin.crba(robot.model, robot.data, q)
ddq = np.linalg.solve(M, (tau - b))  # valid only when no collision
dt, dq = 0.01, 0
dq += ddq * dt
q = pin.integrate(robot.model, q, dq * dt)
#
# M = pin.crba(robot, robot.data, q)
# # # d/dq M(q)
# dcrba = DCRBA(robot)
# dcrba.pre(q)
# Mp = dcrba()
# #
# # d/dvq RNEA(q,vq) = C(q,vq)
# coriolis = Coriolis(robot)
# C = coriolis(q,vq)
# #
# # # d/dq RNEA(q,vq,aq)
# drnea = DRNEA(robot)
# R = drnea(q,vq,aq)
# print('hi')
#
