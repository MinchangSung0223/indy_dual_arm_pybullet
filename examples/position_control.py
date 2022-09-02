import sys
sys.path.append('../src')

import pybullet as p
import time
from math import sin

from kuka import Kuka


duration = 3000
stepsize = 1e-3

robot = Kuka(stepsize)
robot.setControlMode("torque")

for i in range(int(duration/stepsize)):
    if i%1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))
 
    # if i%3000 == 0:
    #     robot.reset()
    #     robot.resetBall()
    #     robot.setControlMode("position")
    #     pos, vel = robot.getJointStates()
    #     target_pos = pos



    #target_pos = robot.solveInverseKinematics([0.5,0.0,0.5],[0,0,0,1])
    #robot.setTargetPositions(target_pos)

    pos, vel = robot.getJointStates()
    acc = [0 for x in pos]
    target_torque = robot.solveInverseDynamics(pos, vel, acc)
    robot.setTargetTorques(target_torque)
    
    robot.step()


    time.sleep(robot.stepsize)