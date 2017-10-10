import VFControl
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import time
from matplotlib.animation import FuncAnimation

def expPathFunc(t):
    #vel = np.array([0,0])
    #return vel
    v = 1
    if (t <= 1):
        theta = np.array([0, 1])
        ang = np.arctan2(theta[1], theta[0])
        vel = v * np.array([np.sin(ang), np.cos(ang)])
    elif (t < 2):
        theta = [1, 0]
        ang = np.arctan2(theta[1], theta[0])
        vel = v * np.array([np.sin(ang), np.cos(ang)])
    elif (t < 3):
        theta = [1, 1]
        ang = np.arctan2(theta[1], theta[0])
        vel = v * np.array([np.sin(ang), np.cos(ang)])
    elif (t < 4):
        theta = [0, 1]
        ang = np.arctan2(theta[1], theta[0])
        vel = v * np.array([np.sin(ang), np.cos(ang)])
    else:
        vel = np.array([0, 0])
    return vel


turnrate = 12
dt = 0.0067
t = 0
r = 1
ua = VFControl.VFUAV(dt)
ua.SetPosition([r * -1.5, 0])
uav_v = 5
theta = 0
vx = uav_v * np.cos(theta)
vy = uav_v * np.sin(theta)
uo = {'vx': vx, 'vy': vy, 'heading': theta}
ua.SetVelocityAndHeading(uo)
ua.bDubinsPathControl = True
ua.bVFControlHeading = False
ua.mTurnrate = turnrate

print(str(ua.GetHeading()))
print(str(ua.GetPosition()))

# sys.exit(1)

cvf = VFControl.CircleVectorField('Gradient')
cvf.bUsePathFunc = True
cvf.velPathFunc = expPathFunc

VF_list = []
for i in range(0, 1000):
    cvf.UpdatePosition(t,dt,ua)
    VF_list.append(cvf.GetVF_XYUV(t,dt,ua))
    ua.UpdateControlFromVF(cvf, t)
    t=t+dt

plt.figure()
plt.ion()
uav_x = []
uav_y = []
axlist = []
figlist = []
for i in range(0, len(ua.mPositionHistory)-1):
    plt.cla()
    vf = VF_list[i]
    Q = plt.quiver(vf['x'],vf['y'],vf['u'],vf['v'])  # , units='width')

    #Q = plt.quiver(Xd, Yd, Udn, Vdn)  # , units='width')
    circle1 = plt.Circle((vf['xc'], vf['yc']), r, color='b', fill=False)
    ax = plt.gca()
    ax.add_artist(plt.scatter(vf['xc'], vf['yc']))
    ax.add_artist(circle1)
    pos = ua.mPositionHistory[i]
    uav_x.append(pos[0])
    uav_y.append(pos[1])
    plt.scatter(uav_x, uav_y)
    plt.axis('equal')
    plt.pause(0.000001)



while True:
    plt.pause(0.05)

sys.exit(1)

