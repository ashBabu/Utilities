import numpy as np
import scipy.optimize as opt
import matplotlib.pyplot as plt


class Overtaking():
    def __init__(self):
        self.V_Ax = 0
        self.V_Ay = 30
        self.x_A0 = 0
        self.y_A0 = 9
        self.x_B0, self.y_B0 = 0, 3
        # self.V_Bx = 0
        self.V_By = 60
        self.r_A, self.r_B = 2, 3
        self.t = np.linspace(0, 1, 100)
   
    def cost(self, V_Bx, dt):
       x_B = self.x_B0 + V_Bx * dt
       y_B = self.y_B0 + self.V_By * dt
       x_A = self.x_A0 + self.V_Ax * dt
       y_A = self.y_A0 + self.V_Ay * dt
       dist = np.sqrt((x_A - x_B)**2 + (y_A - y_B)**2)
       temp = dist - (self.r_B + self.r_A)
       V_A = np.sqrt(V_Bx**2 + self.V_By**2)
       if temp < 0:
           obs_cost = -15*temp - V_Bx
       else:
           obs_cost = 0
       return obs_cost + x_B**2
       
    def call_optimize(self):
       vb0 = 0
       results = np.zeros(100)
       i = 0
       for t in self.t:
           res = opt.minimize(self.cost, vb0, args=t, method='BFGS', options={'maxiter': 150, 'disp': True})
           results[i] = res.x
           vb0 = res.x
           i += 1
       return results

    def animation(self, x_B, y_B, x_A, y_A):
        theta = np.linspace(0, 2 * np.pi, 100)
        v = 40
        cx = np.zeros(v)
        cy = np.linspace(0, v, v)
        right_x = 9 * np.ones(v)
        left_x = -right_x

        for i in range(len(x_A)):
            plt.clf()
            plt.plot(cx, cy, 'k--')
            plt.plot(right_x, cy, 'b')
            plt.plot(left_x, cy, 'b')
            cAx = x_A[i] + overtake.r_A * np.cos(theta)
            cAy = y_A[i] + overtake.r_A * np.sin(theta)

            cBx = x_B[i] + overtake.r_B * np.cos(theta)
            cBy = y_B[i] + overtake.r_A * np.sin(theta)
            plt.plot(x_B[i], y_B[i], 'r*')
            plt.plot(x_A[i], y_A[i], 'b*')
            plt.plot(cAx, cAy)
            plt.plot(cBx, cBy)
            plt.xlim(-v/2, v/2)
            plt.ylim(0, v)
            # plt.axis('equal')
            # plt.savefig("/home/ar0058/Ash/repo/model_predictive_control/src/animation/overtaking/%02d.png" % i)
            plt.pause(0.01)

       
if __name__ == '__main__':
    overtake = Overtaking()
    V_Bx = overtake.call_optimize()
    x_B = overtake.x_B0 + V_Bx * overtake.t
    y_B = overtake.y_B0 + overtake.V_By * overtake.t
    x_A = overtake.x_A0 + overtake.V_Ax * overtake.t
    y_A = overtake.y_A0 + overtake.V_Ay * overtake.t

    overtake.animation(x_B, y_B, x_A, y_A)

    print('hi')
