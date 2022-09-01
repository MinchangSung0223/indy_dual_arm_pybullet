import sys
sys.path.append('../src')

import time
from math import sin

from indy import Indy

duration = 30
stepsize = 1e-3

robot = Indy(stepsize)
robot.setControlMode("torque")

for i in range(int(duration/stepsize)):
    if i%1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))
 
    if i%10000 == 0:
        robot.reset()
        robot.setControlMode("torque")
        target_torque = [0 for i in range(robot.dof)]

    pos, vel = robot.getJointStates()
    target_torque = [0,0,0,0,0,0]

    robot.setTargetTorques(target_torque)
    robot.step()


    time.sleep(robot.stepsize)