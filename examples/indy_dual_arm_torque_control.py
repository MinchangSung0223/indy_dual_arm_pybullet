import sys
sys.path.append('../src')
import numpy as np
import time
from math import sin

from indy_dual_arm import IndyDualArm

duration = 30000
stepsize = 1e-3
OPEN = 0;
CLOSE= 1;
robot = IndyDualArm(stepsize)
robot.setControlMode("torque")
for i in range(int(duration/stepsize)):
    if i%1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))
 
    if i%10000 == 0:
        robot.reset()
        robot.setControlMode("torque")
        target_torque = [0 for i in range(robot.dof)]

    target_torque = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    joint_pos, joint_vel = robot.getJointStates()
    joint_acc = [0 for x in joint_pos]        
    gravity_torque = robot.solveInverseDynamics(joint_pos, joint_vel, joint_acc)
    robot.setTargetTorques(gravity_torque)
    robot.setRightGripperState(OPEN)
    robot.setLeftGripperState(OPEN)
    robot.step()
    time.sleep(robot.stepsize)