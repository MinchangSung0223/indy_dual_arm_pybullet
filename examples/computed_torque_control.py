import sys
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
Kp = np.diagflat([20,20,20,20,20,20])
Kv = np.diagflat([1,1,1,1,1,1])
for i in range(int(duration/timeStep)):
    if i%1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))
 


    q, qdot = robot.getJointStates()
    qddot = [0 for i in q]
    H = robot.solveInverseDynamics(q, qdot, qddot)
    M = robot.getMassMatrix(q);
    e = q_des-q;
    edot = qdot_des-qdot;
    qddot_ref = qddot_des + Kv.dot(edot) + Kp.dot(e)

    tau = M.dot(qddot_ref) +H
    maxF = np.array(robot.max_torque)
    generalized_forces = np.clip(tau, -maxF, maxF)
    
    robot.setTargetTorques(generalized_forces)
    robot.step()
    time.sleep(robot.timeStep)