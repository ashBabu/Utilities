import pybullet as pyb
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as si
import os
from utils import Utils

save_dir = os.path.dirname(os.path.abspath(__file__))
QList = np.load(save_dir+'/data/JointAnglesList.npy', allow_pickle=True)
QList = list(np.transpose(QList, (0, 2, 1)))  # Trajectories should of the shape nT x nDoF having length nDemo
nT, nDoF = QList[0].shape[0], QList[0].shape[1]
VList = [0.1*np.diff(Q, axis=0) for Q in QList]  # velocity (nT - 1) x nDoF
VList = [np.vstack((V, np.zeros(nDoF))) for V in VList]
# StateVector = [np.vstack((q, v)) for q, v in zip(QList, VList)]
timeList = [np.linspace(0, 1, QList[0].shape[0]) for i in range(len(QList))]

traj = dict()
traj['t'] = timeList[0]
traj['q'] = QList[0]
traj['v'] = VList[0]
traj['q0'] = QList[0][0, :]
traj['qf'] = QList[0][-1, :]
traj['v0'] = VList[0][0, :]
traj['vf'] = VList[0][-1, :]


class HermiteTrajectory:
    """This class implementes piecewise Hermite spline to generate interpolation
    that matches both q and v on time grids, acceleration might be discontinuous.
    It use BPoly for implementation.
    """
    def __init__(self, traj, show_traj=False):
        full_time = traj['t']
        plan_time = full_time[-1] - full_time[0]
        # super(HermiteTrajectory, self).__init__(plan_time, traj['q0'], traj['qf'])
        ntmid, dimq = traj['q'].shape
        tgrid = np.zeros(ntmid + 2)
        tgrid[0] = 0
        tgrid[-1] = plan_time
        tgrid[1:-1] = traj['tmid']
        xvals = np.zeros((ntmid + 2, 2))
        bpolys = []
        for i in range(dimq):
            xvals[:, 0] = np.concatenate(([traj['q0'][i]], traj['q'][:, i], [traj['qf'][i]]))
            xvals[:, 1] = np.concatenate(([traj['v0'][i]], traj['v'][:, i], [traj['vf'][i]]))
            poly = si.BPoly.from_derivatives(tgrid, xvals, orders=3)
            bpolys.append(poly)
        self.bpolys = bpolys
        if show_traj:
            fig, ax = plt.subplots(3, 3)
            ax[0][0].plot(traj['tmid'], traj['q'])
            ax[1][0].plot(traj['tmid'], traj['v'])
            ax[2][0].plot(traj['tmid'], traj['a'])
            sample_t = np.linspace(0, plan_time, 100)
            ax[0][1].plot(sample_t, self.getq(sample_t))
            ax[1][1].plot(sample_t, self.getv(sample_t))
            ax[0][2].plot(traj['tmid'], self.getq(traj['tmid']) - traj['q'])
            ax[1][2].plot(traj['tmid'], self.getv(traj['tmid']) - traj['v'])
            fig.tight_layout()
            plt.show()

    def getq(self, t):
        return np.array([bpoly(t) for bpoly in self.bpolys]).T

    def getv(self, t):
        return np.array([bpoly(t, 1) for bpoly in self.bpolys]).T


verbose = False
delta_t = 0.01
jointIndex = [0, 1, 2, 3, 4, 5, 6, 8, 9]  # 7 is a fixed joint
util = Utils()
physicsClient = pyb.connect(pyb.GUI)  # or pyb.DIRECT for non-graphical version
pyb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
pyb.setPhysicsEngineParameter(enableFileCaching=1)
pyb.setGravity(0, 0, 0)
# planeId = pyb.loadURDF("space.urdf")
cubeStartPos = [0, 0, 0]
# cubeStartPos = [8.463e-02, -4.806e-01, 0]  # [0, 0, 2]
cubeStartOrientation = pyb.getQuaternionFromEuler([0, 0, 0])
# boxId = pyb.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
# boxId0 = pyb.loadURDF("ur5_robot.urdf", cubeStartPos, cubeStartOrientation, flags=1, globalScaling=3, useFixedBase=0)
spaceRobotId = pyb.loadURDF("free_floating_7dof.urdf", cubeStartPos, cubeStartOrientation, flags=pyb.URDF_USE_SELF_COLLISION, globalScaling=.5, useFixedBase=0)
# spaceRobotId = pyb.loadMJCF("/home/ash/Ash/MuJoCo/xml/spaceRobot.xml")
# spaceRobotId = pyb.loadMJCF("/home/ash/Ash/MuJoCo/xml/mjmodel.mjb")
"""
pyb.getEulerFromQuaternion((0, 0, 0, 1)) Out[9]: (0.0, -0.0, 0.0)
pyb.getEulerFromQuaternion((0, 0, 1, 0)) Out[10]: (0.0, -0.0, 3.141592653589793)
pyb.getEulerFromQuaternion((0, 1, 0, 0)) Out[11]: (3.141592653589793, -0.0, 3.141592653589793)
pyb.getEulerFromQuaternion((1, 0, 0, 0)) Out[12]: (3.141592653589793, -0.0, 0.0)
"""
numJoints = pyb.getNumJoints(spaceRobotId)  # returns 10. 7DoF robot + 1 fixed joint to connect robot to hand + 2 finger joints
endEffectorId = numJoints - 3
q = np.zeros(numJoints-1)
qdot = np.zeros_like(q)
js = pyb.getJointStates(spaceRobotId, jointIndex)
for i in range(numJoints-1):
    q[i] = pyb.getJointState(spaceRobotId, i)[0]
    qdot[i] = pyb.getJointState(spaceRobotId, i)[1]

print(pyb.getLinkState(spaceRobotId, endEffectorId, 1, 1))
ikSolver = 0
useRealTimeSimulation = 0
pyb.setRealTimeSimulation(useRealTimeSimulation)
t = 0.
hasPrevPose = 0
trailDuration = 0  # 0 for permanent

ls = pyb.getLinkState(spaceRobotId, endEffectorId)
end = [-0.5, 0, 1]
start = np.array(ls[4])
path = util.discretize(start, end, step_size=delta_t)
pyb.addUserDebugLine(ls[4], end, [0, 0, 0.3], 1, trailDuration)
# jd = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 0.5]
jd=[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,]
#
# jointPoses = pyb.calculateInverseKinematics(spaceRobotId, endEffectorId,
#                                             end, jointDamping=jd, solver=ikSolver)
jointPoses = pyb.calculateInverseKinematics(spaceRobotId, endEffectorId,
                                            end, jointDamping=jd)
jointPosesPath = np.zeros((path.shape[0], len(jointPoses)))
for i in range(jointPosesPath.shape[0]):
    goal = path[i, :]
    jointPosesPath[i, :] = pyb.calculateInverseKinematics(spaceRobotId, endEffectorId, goal)
pdControl = 1
while pdControl:
    # print(i)
    pyb.setJointMotorControlArray(bodyIndex=spaceRobotId,
                                jointIndices=[0, 1, 2, 3, 4, 5, 6],
                                controlMode=pyb.POSITION_CONTROL,
                                targetPositions=jointPoses[:7],
                                targetVelocities=np.zeros(7),
                                forces=0 * np.ones(7),
                                positionGains=00 * np.ones(7),
                                velocityGains=0 * np.ones(7))
    # How to find Gains, computed torque?
    pyb.stepSimulation()
    time.sleep(0.01)

""" Torque Control """
pyb.setJointMotorControlArray(spaceRobotId,
                              jointIndex,
                              pyb.VELOCITY_CONTROL,
                              forces=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
for i in range(numJoints):
    # continue
    pyb.changeDynamics(spaceRobotId, i, linearDamping=0, angularDamping=0, jointDamping=0)

q_discretized = util.discretize(q, jointPoses, step_size=delta_t)
tme = np.linspace(0, 10, q_discretized.shape[0])
tme1 = np.linspace(0, 10, jointPosesPath.shape[0])
cs = si.CubicSpline(tme, q_discretized)
cs1 = si.CubicSpline(tme1, jointPosesPath)
qdot_discretized = cs(tme, 1)
qddot_discretized = cs(tme, 2)

# q_tor = [[0.] * steps, [0.] * steps]
# torque = -500*np.ones(9)
t = 0
q_t, qdot_t, qddot_t = q_discretized[t], qdot_discretized[t], qddot_discretized[t]
# trq = pyb.calculateInverseDynamics(spaceRobotId, q_t, qddot_t, qddot_t)
# trq = pyb.calculateInverseDynamics(spaceRobotId, q_t, qddot_t, qddot_t)
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
dir = '/home/ash/Ash/repo/ash_pybullet'

urdf_filename = dir + '/free_floating_7dof.urdf'
robot = RobotWrapper.BuildFromURDF(urdf_filename, [dir])
trq = np.zeros_like(q_discretized)
for t in range(q_discretized.shape[0]):
    q_t, qdot_t, qddot_t = q_discretized[t], qdot_discretized[t], qddot_discretized[t]
    b = pin.rnea(robot.model, robot.data, q_t, qdot_t, np.zeros_like(q_t))
    # compute mass matrix M
    M = pin.crba(robot.model, robot.data, q_t)
    trq[t, :] = M @ qddot_t + b
# for t in range(len(tme)):
while t <= len(tme):
    # q_t, qdot_t, qddot_t = jointPosesPath[t], cs1(tme1, 1)[t], cs1(tme1, 2)[t]
    q_t, qdot_t, qddot_t = q_discretized[t], qdot_discretized[t], qddot_discretized[t]
    # torque = pyb.calculateInverseDynamics(spaceRobotId, q_t, qddot_t, qddot_t)
    torque = pin.rnea(robot.model, robot.data, q_t, qddot_t, qddot_t)
    # torque = trq[t, :]
    # print(trq1 - trq[t, :])
    # q_tor[0][i] = torque[0]
    # q_tor[1][i] = torque[1]
    if (verbose):
        print("torque=")
        print(torque)

    # Set the Joint Torques:
    pyb.setJointMotorControlArray(spaceRobotId,
                                     jointIndex,
                                     pyb.TORQUE_CONTROL,
                                     forces=[torque[0], torque[1], torque[2], torque[3], torque[4], torque[5], torque[6], torque[7], torque[8]])
    # pyb.setJointMotorControlArray(bodyIndex=spaceRobotId,
    #                               jointIndices=jointIndex,
    #                               controlMode=pyb.POSITION_CONTROL,
    #                               targetPositions=q_t,
    #                               targetVelocities=qdot_t,
    #                               forces=20 * np.ones(len(jointIndex)),
    #                               positionGains=100 * np.ones(len(jointIndex)),
    #                               velocityGains=100 * np.ones(len(jointIndex)))
    # Step Simulation
    pyb.stepSimulation()
    if t == len(tme):
        t = len(tme)
    time.sleep(0.01)
pyb.disconnect()
# computed Torque control, feedforward compensation