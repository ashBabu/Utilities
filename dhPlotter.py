import numpy as np
import matplotlib.pyplot as plt

class DHPlotter:

    def __init__(self, DH=None):  # DH should be a dict containing a, d, alpha
        if not DH:
            self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0])
            self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107])
            self.alpha = np.array([0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0])
        else:
            self.a = DH['a']
            self.d = DH['d']
            self.alpha = DH['alpha']
        self.numJoints = len(self.a)-1
        self.T_desired =[]

    def fwd_kin(self, q=None,):   # forward kinematics, # Expects a 7 x 1 values for joint_values
        t, T_joint, Ti = np.eye(4), np.zeros((self.numJoints + 1, 4, 4)), np.zeros((self.numJoints + 1, 4, 4))
        q = np.insert(q, len(q), 0.0, axis=0)  # added 0 at the end for the fixed joint
        for i in range(q.shape[0]):
            T = np.array([[np.cos(q[i]), -np.sin(q[i]), 0, self.a[i]],
                          [np.sin(q[i]) * np.cos(self.alpha[i]), np.cos(q[i]) * np.cos(self.alpha[i]), -np.sin(self.alpha[i]), -np.sin(self.alpha[i]) * self.d[i]],
                          [np.sin(q[i]) * np.sin(self.alpha[i]), np.cos(q[i]) * np.sin(self.alpha[i]), np.cos(self.alpha[i]), np.cos(self.alpha[i]) * self.d[i]],
                          [0, 0, 0, 1]], dtype='float')
            t = t @ T
            # Ti[i, :, :] = T
            T_joint[i, :, :] = t
        return t, T_joint

    def plotter(self, ax, T_joint, lgnd, color='r'):
        x, y, z = 0, 0, 0
        for i in range(len(self.a)):
            ax.plot([x, T_joint[i, 0, 3]], [y, T_joint[i, 1, 3]], [z, T_joint[i, 2, 3]], color, label=lgnd)
            ax.scatter(T_joint[i, 0, 3], T_joint[i ,1, 3], T_joint[i, 2, 3], 'gray')
            x, y, z = T_joint[i, 0, 3], T_joint[i, 1, 3], T_joint[i, 2, 3]
            plt.xlabel('X')
            plt.ylabel('Y')
            # plt.axis('equal')
            scale = 0.4
            # ax.set_xlim(- 1 *scale, 1* scale)
            # ax.set_ylim(-1 * scale, 1 * scale)
            # ax.set_zlim(0, 1)


if __name__ == '__main__':

    DH = dict()
    DH['alpha'] = np.array([0, np.pi/2, 0, 0, 0, 0], dtype=np.float32)
    DH['d'] = np.array([0.1, 0., 0., 0., 0., 0.1], dtype=np.float32)
    DH['a'] = np.array([0., 0., 0.3, 0.3, 0.3, 0], dtype=np.float32)
    q = np.random.randn(5)
    dh = DHPlotter(DH)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    t, T_joint = dh.fwd_kin(q)
    dh.plotter(ax=ax, T_joint=T_joint, lgnd='franka')
    plt.show()