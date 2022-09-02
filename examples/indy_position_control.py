import sys
from pyScurveGenerator import *
sys.path.append('../src')
import numpy as np
import pybullet as p
import time
import matplotlib.pyplot as plt
from math import sin

from indy_ball import Indy
duration = 3000
stepsize = 1e-3
robot = Indy(stepsize)
robot.setControlMode("position")

traj = Trajectory();
traj.so = 0;
traj.vo = 10;
traj.ao = 0;
traj.sf = 1;
traj.vf = 18;
traj.af = 5;
traj.vmax = 10;
traj.amax = 200;
traj.dmax = 200;
traj.j = 1000;

sg = ScurveGenerator([traj])
sg.syncTime();
s_list1      = np.array(sg.getSlist(sg.getTraj(0),stepsize/10))
ds_list1    = np.array(sg.getdSlist(sg.getTraj(0),stepsize/10))
dds_list1    = np.array(sg.getddSlist(sg.getTraj(0),stepsize/10))
ddds_list1   = np.array(sg.getdddSlist(sg.getTraj(0),stepsize/10))


traj2 = Trajectory();
traj2.so = 0;
traj2.vo = -10;
traj2.ao = -5;
traj2.sf = 1;
traj2.vf = 18;
traj2.af = 7;
traj2.vmax = 20;
traj2.amax = 200;
traj2.dmax = 200;
traj2.j = 2000;

sg = ScurveGenerator([traj2])
sg.printAllTrajectory()

sg.syncTime();
s_list2      = np.array(sg.getSlist(sg.getTraj(0),stepsize/10))
ds_list2    = np.array(sg.getdSlist(sg.getTraj(0),stepsize/10))
dds_list2    = np.array(sg.getddSlist(sg.getTraj(0),stepsize/10))
ddds_list2   = np.array(sg.getdddSlist(sg.getTraj(0),stepsize/10))


s_list = []
x_list = []
ds_list = []
dds_list = []
ddds_list = []
x_start = 0.5
y_start = 0.0
z_start = 0.3
x_end = 0.7
y_end = 0.0
z_end = 0.3
for j in range(len(s_list1)):
    s_list.append(s_list1[j]*(x_end-x_start))  
    x_list.append(x_start + s_list1[j]*(x_end-x_start))
    ds_list.append(ds_list1[j]*(x_end-x_start)) 
    dds_list.append(dds_list1[j]*(x_end-x_start)) 
    ddds_list.append(ddds_list1[j]*(x_end-x_start))  
x_start = 0.7
y_start = 0.0
z_start = 0.3
x_end = 0.5
y_end = 0.0
z_end = 0.3
for j in range(len(s_list2)):
    s_list.append(s_list2[j]*(x_end-x_start)-(x_end-x_start))  
    x_list.append(x_start + s_list2[j]*(x_end-x_start))
    ds_list.append(ds_list2[j]*(x_end-x_start)) 
    dds_list.append(dds_list2[j]*(x_end-x_start)) 
    ddds_list.append(ddds_list2[j]*(x_end-x_start))  
x_start = 0.5
y_start = 0.0
z_start = 0.3
x_end = 0.5
y_end = 0.0
z_end = 0.3
target_pos = robot.solveInverseKinematics([x_start,y_start,z_start],[1,0,0,0])
for i in range(2000):
    target_pos = robot.solveInverseKinematics([x_start,y_start,z_start],[1,0,0,0])
    robot.setTargetPositions(target_pos)
    robot.step()
    time.sleep(robot.stepsize)
print("START")
p=robot.p 
t_list = []

y_list = []
z_list = []
dx_list = []
dy_list = []
dz_list = []
ddx_list = []
ddy_list = []
ddz_list = []
dddx_list = []

diff_x_list = []
diff_dx_list = []
diff_ddx_list = []

prev_dx = 0;
prev_ddx = 0;
prev_filtered_ddx = 0;
prev_filtered_dddx = 0;
alpha = 0.9
prev_diff_x=s_list[0];
for i in range(len(s_list)):
    x =x_list[i]
    y = 0.0
    z =0.3
    target_pos = robot.solveInverseKinematics([x,y,z],[1,0,0,0])
    robot.setTargetPositions(target_pos)
    result = p.getLinkState(robot.robot_id,6,computeLinkVelocity=1,computeForwardKinematics=1)
    link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
    t_list.append(i*(stepsize/10))
    y_list.append(link_trn[1])

    z_list.append(link_trn[2])
    dx_list.append(link_vt[0]*10)
    dy_list.append(link_vt[1])
    dz_list.append(link_vt[2])

    diff_x_list.append((prev_diff_x - s_list[i])/(stepsize/10))
    prev_diff_x = s_list[i];

    new_ddx = (link_vt[0]-prev_dx)/(stepsize/10);
    filtered_ddx = alpha*prev_filtered_ddx + (1-alpha)*new_ddx
    ddx_list.append(filtered_ddx*10)
    new_dddx = (ddx_list[i]-prev_ddx)/(stepsize/10);
    filtered_dddx = alpha*prev_filtered_dddx + (1-alpha)*new_dddx
    dddx_list.append(filtered_dddx*10)
    prev_dx =link_vt[0]
    prev_filtered_ddx = filtered_ddx
    prev_filtered_dddx = filtered_dddx
    prev_ddx = ddx_list[i];

    robot.step()
    time.sleep(robot.stepsize)
fig, ax = plt.subplots(4,1)
ax[0].plot(t_list,x_list,t_list,np.array(s_list))
ax[1].plot(t_list,dx_list,t_list,np.array(ds_list))
ax[2].plot(t_list,ddx_list,t_list,np.array(dds_list))
ax[3].plot(t_list,dddx_list,t_list,np.array(ddds_list))
plt.show()