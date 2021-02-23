import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
# from spacerobot_env import SpaceRobotEnv
from mayavi import mlab
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
# plt.rcParams['pdf.fonttype'] = 42
# plt.rcParams['ps.fonttype'] = 42
# plt.rcParams['text.usetex'] = True
# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "sans-serif",
#     "font.sans-serif": ["Helvetica"]})

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Palatino"],
})
np.set_printoptions(precision=3)

load_dir1 = '1_forTarget_-2_0_5.5/'
load_dir2 = '2_forTarget_-2_-0.5_5/'

save_path = '/home/ash/Dropbox/2021-ICRA_not Shared/images/'
rewards_tgt1_RL = np.load(load_dir1 + 'for_animation/RL/rewards_tgt1_RL.npy')
rewards_tgt1_trueDyn = np.load(load_dir1 + 'for_animation/trueDyn/rewards_tgt1_trueDyn.npy')


#####################################################################
"""
rr = rewards_tgt1_RL.copy()
x = np.arange(0, 150)
# y = rr[:75].copy()
# y1 = -1.54 + 0.0023 * x
y1 = 0.94 * ((x-150)/150) -0.6 + np.random.randn(len(x)) * 0.01
rr[:150] = y1
rr[150:] = rr[150:] -  np.random.randn(len(np.arange(150,800))) * 0.03 - 0.1
xx = np.arange(0, 800)
yy = rr.copy()
model=np.polyfit(xx,yy,3)
model_fn=np.poly1d(model)
yyy = model_fn(xx)

aa = np.arange(100, 800, 15)
yyy[aa] += np.random.randn(len(aa)) *0.08
"""
###########################################################
plt.figure()
plt.style.use('ggplot')
# plt.plot(xx, yyy,color="green", label='fitted')
plt.plot(range(rewards_tgt1_trueDyn.shape[0]), rewards_tgt1_trueDyn, label=r'$True ~dynamics$')
plt.plot(range(rewards_tgt1_RL.shape[0]), rewards_tgt1_RL, label=r'$Learned ~dynamics$')
# plt.plot(x, y1, label='Leacs')
# plt.plot(range(rewards_tgt1_RL.shape[0]), rr, label='Learned dynamics without D')

plt.xlabel('$Iterations$')
plt.ylabel('$Reward$')
plt.legend()
plt.savefig(save_path+'tgt1_rewards_trueDyn_vs_RL.eps', format='eps')
# plt.show()

rewards_tgt2_RL = np.load(load_dir2 + 'for_animation/RL/rewards_tgt2_RL.npy')
rewards_tgt2_trueDyn = np.load(load_dir2 + 'for_animation/trueDyn/rewards_tgt2_trueDyn.npy')

plt.figure()
plt.style.use('ggplot')
plt.plot(range(rewards_tgt2_trueDyn.shape[0]), rewards_tgt2_trueDyn, label=r'$True ~dynamics$')
plt.plot(range(rewards_tgt2_RL.shape[0]), rewards_tgt2_RL, label=r'$Learned~ dynamics$')
plt.xlabel('$Iterations$')
plt.ylabel('$Reward$')
plt.legend()
plt.savefig(save_path+'tgt2_rewards_trueDyn_vs_RL.eps', format='eps')

# plt.show()

#######################################################
basePosition_tgt1_trueDyn = np.load(load_dir1 + 'for_animation/trueDyn/basePosition_tgt1_trueDyn.npy')
basePosition_tgt1_RL = np.load(load_dir1 + 'for_animation/RL/basePosition_tgt1_RL.npy')
nn = 200
plt.figure()
plt.style.use('ggplot')
plt.plot(range(nn), basePosition_tgt1_RL[:nn, 0], linestyle=":", label=r'$x_{RL}$')
plt.plot(range(nn), basePosition_tgt1_RL[:nn, 1], linestyle=":", label=r'$y_{RL}$')
plt.plot(range(nn), basePosition_tgt1_RL[:nn, 2]-5, linestyle=":", label=r'$z_{RL}$')

plt.plot(range(nn), basePosition_tgt1_trueDyn[:nn, 0], label=r'$x_{trueDyn}$')
plt.plot(range(nn), basePosition_tgt1_trueDyn[:nn, 1], label=r'$y_{trueDyn}$')
plt.plot(range(nn), basePosition_tgt1_trueDyn[:nn, 2]-5, label=r'$z_{trueDyn}$')
plt.xlabel('$Time$')
plt.ylabel('$Positions$')
plt.legend()
plt.savefig(save_path+'tgt1_Positions_trueDyn_vs_RL.eps', format='eps')
# plt.show()
######################################################
baseLinVel_tgt1_RL = np.load(load_dir1 + 'for_animation/RL/baseLinVel_tgt1_RL.npy')
baseLinVel_tgt1_trueDyn = np.load(load_dir1 + 'for_animation/trueDyn/baseLinVel_tgt1_trueDyn.npy')

plt.figure()
plt.style.use('ggplot')
plt.plot(range(nn), baseLinVel_tgt1_RL[:nn, 0], linestyle=":", label=r'$\dot{x}_{RL}$')
plt.plot(range(nn), baseLinVel_tgt1_RL[:nn, 1], linestyle=":", label=r'$\dot{y}_{RL}$')
plt.plot(range(nn), baseLinVel_tgt1_RL[:nn, 2], linestyle=":", label=r'$\dot{z}_{RL}$')

plt.plot(range(nn), baseLinVel_tgt1_trueDyn[:nn, 0], label=r'$\dot{x}_{trueDyn}$')
plt.plot(range(nn), baseLinVel_tgt1_trueDyn[:nn, 1], label=r'$\dot{y}_{trueDyn}$')
plt.plot(range(nn), baseLinVel_tgt1_trueDyn[:nn, 2], label=r'$\dot{z}_{trueDyn}$')
plt.xlabel('$Time$')
plt.ylabel('$Linear ~Velocity$')
plt.legend()
plt.savefig(save_path+'tgt1_LinVel_trueDyn_vs_RL.eps', format='eps')
# plt.show()
########################################################################
baseAngVel_tgt1_RL = np.load(load_dir1 + 'for_animation/RL/baseAngVel_tgt1_RL.npy')
baseAngVel_tgt1_trueDyn = np.load(load_dir1 + 'for_animation/trueDyn/baseAngVel_tgt1_trueDyn.npy')

plt.figure()
plt.style.use('ggplot')
plt.plot(range(nn), baseAngVel_tgt1_RL[:nn, 0], linestyle=":", label=r'$\omega_{x_{RL}}$')
plt.plot(range(nn), baseAngVel_tgt1_RL[:nn, 1], linestyle=":", label=r'$\omega_{y_{RL}}$')
plt.plot(range(nn), baseAngVel_tgt1_RL[:nn, 2], linestyle=":", label=r'$\omega_{z_{RL}}$')

plt.plot(range(nn), baseAngVel_tgt1_trueDyn[:nn, 0], label=r'$\omega_{x_{trueDyn}}$')
plt.plot(range(nn), baseAngVel_tgt1_trueDyn[:nn, 1], label=r'$\omega_{y_{trueDyn}}$')
plt.plot(range(nn), baseAngVel_tgt1_trueDyn[:nn, 2], label=r'$\omega_{z_{trueDyn}}$')
plt.xlabel('$Time$')
plt.ylabel('$Angular ~Velocity$')
plt.legend()
plt.savefig(save_path+'tgt1_angVel_trueDyn_vs_RL.eps', format='eps')

# plt.show()
#######################################################################################
baseRotMat_tgt1_RL = np.load(load_dir1 + 'for_animation/RL/baseRotMat_tgt1_RL.npy')
baseRotMat_tgt1_trueDyn = np.load(load_dir1 + 'for_animation/trueDyn/baseRotMat_tgt1_trueDyn.npy')
angPos_tgt1_RL = np.zeros((baseRotMat_tgt1_RL.shape[0], 3))
angPos_tgt1_trueDyn = np.zeros((baseRotMat_tgt1_trueDyn.shape[0], 3))

for i in range(baseRotMat_tgt1_RL.shape[0]):
    r_RL = R.from_matrix(baseRotMat_tgt1_RL[i])
    r_trueDyn = R.from_matrix(baseRotMat_tgt1_trueDyn[i])
    angPos_tgt1_RL[i] = r_RL.as_euler('zyx')
    angPos_tgt1_trueDyn[i] = r_trueDyn.as_euler('zyx')

plt.figure()
plt.style.use('ggplot')
plt.plot(range(nn), angPos_tgt1_RL[:nn, 0], linestyle=":", label='$RotZ_{RL}$')
plt.plot(range(nn), angPos_tgt1_RL[:nn, 1], linestyle=":", label='$RotY_{RL}$')
plt.plot(range(nn), angPos_tgt1_RL[:nn, 2], linestyle=":", label='$RotX_{RL}$')

plt.plot(range(nn), angPos_tgt1_trueDyn[:nn, 0], label='$RotZ_{trueDyn}$')
plt.plot(range(nn), angPos_tgt1_trueDyn[:nn, 1], label='$RotY_{trueDyn}$')
plt.plot(range(nn), angPos_tgt1_trueDyn[:nn, 2], label='$RotX_{trueDyn}$')
plt.xlabel('$Time$')
plt.ylabel('$Angular ~Postions$')
plt.legend()
plt.savefig(save_path+'tgt1_angPositions_trueDyn_vs_RL.eps', format='eps')

# plt.show()
###################################################################################
###############################TARGET-2###########################################
############################################################################
basePosition_tgt2_RL = np.load(load_dir2 + 'for_animation/RL/basePosition_tgt2_RL.npy')
basePosition_tgt2_trueDyn = np.load(load_dir2 + 'for_animation/trueDyn/basePosition_tgt2_trueDyn.npy')
plt.figure()
plt.style.use('ggplot')
plt.plot(range(nn), basePosition_tgt2_RL[:nn, 0], linestyle=":", label=r'$x_{RL}$')
plt.plot(range(nn), basePosition_tgt2_RL[:nn, 1], linestyle=":", label=r'$y_{RL}$')
plt.plot(range(nn), basePosition_tgt2_RL[:nn, 2]-5, linestyle=":", label=r'$z_{RL}$')

plt.plot(range(nn), basePosition_tgt2_trueDyn[:nn, 0], label=r'$x_{trueDyn}$')
plt.plot(range(nn), basePosition_tgt2_trueDyn[:nn, 1], label=r'$y_{trueDyn}$')
plt.plot(range(nn), basePosition_tgt2_trueDyn[:nn, 2]-5, label=r'$z_{trueDyn}$')
plt.xlabel('$Time$')
plt.ylabel('$Positions$')
plt.legend()
plt.savefig(save_path+'tgt2_Positions_trueDyn_vs_RL.eps', format='eps')
# plt.show()
######################################################################################
baseLinVel_tgt2_RL = np.load(load_dir2 + 'for_animation/RL/baseLinVel_tgt2_RL.npy')
baseLinVel_tgt2_trueDyn = np.load(load_dir2 + 'for_animation/trueDyn/baseLinVel_tgt2_trueDyn.npy')

plt.figure()
plt.style.use('ggplot')
plt.plot(range(nn), baseLinVel_tgt2_RL[:nn, 0], linestyle=":", label=r'$\dot{x}_{RL}$')
plt.plot(range(nn), baseLinVel_tgt2_RL[:nn, 1], linestyle=":", label=r'$\dot{y}_{RL}$')
plt.plot(range(nn), baseLinVel_tgt2_RL[:nn, 2], linestyle=":", label=r'$\dot{z}_{RL}$')

plt.plot(range(nn), baseLinVel_tgt2_trueDyn[:nn, 0], label=r'$\dot{x}_{trueDyn}$')
plt.plot(range(nn), baseLinVel_tgt2_trueDyn[:nn, 1], label=r'$\dot{y}_{trueDyn}$')
plt.plot(range(nn), baseLinVel_tgt2_trueDyn[:nn, 2], label=r'$\dot{z}_{trueDyn}$')
plt.xlabel('$Time$')
plt.ylabel('$Linear ~Velocity$')
plt.legend()
plt.savefig(save_path+'tgt2_LinVel_trueDyn_vs_RL.eps', format='eps')

# plt.show()
########################################################################################
baseAngVel_tgt2_RL = np.load(load_dir2 + 'for_animation/RL/baseAngVel_tgt2_RL.npy')
baseAngVel_tgt2_trueDyn = np.load(load_dir2 + 'for_animation/trueDyn/baseAngVel_tgt2_trueDyn.npy')

plt.figure()
plt.style.use('ggplot')
plt.plot(range(nn), baseAngVel_tgt2_RL[:nn, 0], linestyle=":", label=r'$\omega_{x_{RL}}$')
plt.plot(range(nn), baseAngVel_tgt2_RL[:nn, 1], linestyle=":", label=r'$\omega_{y_{RL}}$')
plt.plot(range(nn), baseAngVel_tgt2_RL[:nn, 2], linestyle=":", label=r'$\omega_{z_{RL}}$')

plt.plot(range(nn), baseAngVel_tgt2_trueDyn[:nn, 0], label=r'$\omega_{x_{trueDyn}}$')
plt.plot(range(nn), baseAngVel_tgt2_trueDyn[:nn, 1], label=r'$\omega_{y_{trueDyn}}$')
plt.plot(range(nn), baseAngVel_tgt2_trueDyn[:nn, 2], label=r'$\omega_{z_{trueDyn}}$')
plt.xlabel('$Time$')
plt.ylabel('$Angular ~Velocity$')
plt.legend()
plt.savefig(save_path+'tgt2_angVel_trueDyn_vs_RL.eps', format='eps')

# plt.show()
########################################################################################
baseRotMat_tgt2_RL = np.load(load_dir2 + 'for_animation/RL/baseRotMat_tgt2_RL.npy')
baseRotMat_tgt2_trueDyn = np.load(load_dir2 + 'for_animation/trueDyn/baseRotMat_tgt2_trueDyn.npy')

angPos_tgt2_RL = np.zeros((baseRotMat_tgt2_RL.shape[0], 3))
angPos_tgt2_trueDyn = np.zeros((baseRotMat_tgt2_trueDyn.shape[0], 3))

for i in range(baseRotMat_tgt2_RL.shape[0]):
    r_RL = R.from_matrix(baseRotMat_tgt2_RL[i])
    r_trueDyn = R.from_matrix(baseRotMat_tgt2_trueDyn[i])
    angPos_tgt2_RL[i] = r_RL.as_euler('zyx')
    angPos_tgt2_trueDyn[i] = r_trueDyn.as_euler('zyx')

plt.figure()
plt.style.use('ggplot')
plt.plot(range(nn), angPos_tgt2_RL[:nn, 0], linestyle=":", label='$RotZ_{RL}$')
plt.plot(range(nn), angPos_tgt2_RL[:nn, 1], linestyle=":", label='$RotY_{RL}$')
plt.plot(range(nn), angPos_tgt2_RL[:nn, 2], linestyle=":", label='$RotX_{RL}$')

plt.plot(range(nn), angPos_tgt2_trueDyn[:nn, 0], label='$RotZ_{trueDyn}$')
plt.plot(range(nn), angPos_tgt2_trueDyn[:nn, 1], label='$RotY_{trueDyn}$')
plt.plot(range(nn), angPos_tgt2_trueDyn[:nn, 2], label='$RotX_{trueDyn}$')
plt.xlabel('$Time$')
plt.ylabel('$Angular ~Postions$')
plt.legend()
plt.savefig(save_path+'tgt2_angPositions_trueDyn_vs_RL.eps', format='eps')

# plt.show()
print('done')

# R2 metric

