import sys
from modern_robotics import *
sys.path.append('../src')

import time
from math import sin
import numpy as np
from indy import Indy

duration = 30
timeStep = 1e-3

robot = Indy(timeStep)
robot.setControlMode("torque")
q_des=np.array([0, 0, 0, 0, 0, 0])
qdot_des=np.array([0, 0, 0, 0, 0, 0])
qddot_des = np.array([0 for i in q_des])
Kp = 1500.0
Kd = Kp /20
Ki = Kp /100
Xd = np.array([[-1.00000000e+00 , 0.00000000e+00 ,-7.34641021e-06  ,5.54550568e-01],
 [ 0.00000000e+00 , 1.00000000e+00  ,0.00000000e+00 ,-1.86500000e-01],
 [ 7.34641021e-06 , 0.00000000e+00 ,-1.00000000e+00  ,3.07207844e+00],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]])

Vd = np.array([0,0,0,0,0,0]).T
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

    invX = TransInv(X)
    Xe = se3ToVec(MatrixLog6(invX @ Xd))
    Ve = Adjoint(invX @ Xd)@Vd - V

    tau = J.T@(Kp*Xe+Kd*Ve)+M@qddot + H
    maxF = np.array(robot.max_torque)

    generalized_forces = np.clip(tau, -maxF*100, maxF*100)
    #print(generalized_forces)
    print(Xe[0])
    robot.setTargetTorques(tau)
    robot.step()
    time.sleep(robot.timeStep)