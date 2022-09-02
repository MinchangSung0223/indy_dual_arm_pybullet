import sys
from modern_robotics import *
sys.path.append('../src')

import time
from math import sin
import numpy as np
from indy import Indy
from math import *

duration = 30
timeStep = 1e-3

robot = Indy(timeStep)
robot.setControlMode("torque")
q_des=np.array([0, 0, 0, 0, 0, 0])
qdot_des=np.array([0, 0, 0, 0, 0, 0])
qddot_des = np.array([0 for i in q_des])
Ka = 60*np.eye(6)
Kp = np.diag([2,2,2,200,200,200])
Kd = Kp/6.0
Ki = Kp/100.0
Xd = np.array([[-1.00000000e+00,  0.00000000e+00, -7.34641021e-06,  4.54540812e-01],
 [ 0.00000000e+00 , 1.00000000e+00  ,0.00000000e+00 ,-1.86500000e-01],
 [ 7.34641021e-06 , 0.00000000e+00 ,-1.00000000e+00  ,4.16078436e-01],
 [ 0.00000000e+00  ,0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]])

Vd = np.array([0,0,0,0,0,0]).T
Sume = np.array([0,0,0,0,0,0]).T
prev_Xe = np.array([0,0,0,0,0,0]).T
for i in range(int(duration/timeStep)):
    if i%1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))
 


    q, qdot = robot.getJointStates()
    qddot = [0 for i in q]
    H = robot.solveInverseDynamics(q, qdot, qddot)
    M = robot.getMassMatrix(q);
    J = robot.getJacobianMatrix(q);

   
    V = J@qdot
    X = robot.getFK(q);
    Xd[0,3] = 0.45+0.1*cos(0.5*pi*i*timeStep)
    Xd[1,3] = 0.0+0.1*sin(0.5*pi*i*timeStep)

    invX = TransInv(X)
    Xe = se3ToVec(MatrixLog6(invX @ Xd))

    Ve = Adjoint(invX @ Xd)@Vd - V
    Sume = Sume + Xe*timeStep
    #tau = M@qddot + H
   
    #tau = J.T@(Ka*(Kp@Xe+Kd@Ve+Ki@Sume)) + H
    tau = J.T@(Ka@(Kp@Xe+Kd@Ve+Ki@Sume))+ M@qddot+ H
    maxF = np.array(robot.max_torque)
    prev_Xe = Xe;
    generalized_forces = np.clip(tau, -maxF, maxF)
    print(X-Xd)
    #print(Xe[0],Xe[1])
    robot.setTargetTorques(generalized_forces)
    robot.step()
    time.sleep(robot.timeStep)