import pybullet as p
import time
class IndyDualArm:
    def __init__(self, stepsize=1e-3, realtime=0):
        self.t = 0.0
        self.stepsize = stepsize
        self.realtime = realtime

        self.control_mode = "torque" 

        self.position_control_gain_p = [0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]
        self.position_control_gain_d = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
        self.max_torque = [100,100,100,100,100,100,100,100,100,100,100,100,100,100]

        self.gripper_position_control_gain_p = [0.05,0.05,0.05,0.05];
        self.gripper_position_control_gain_d = [0.5,0.5,0.5,0.5];
        self.gripper_max_torque = [5,5,5,5];

        # connect pybullet
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=89.99, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.5])

        p.resetSimulation()
        p.setTimeStep(self.stepsize)
        p.setRealTimeSimulation(self.realtime)
        p.setGravity(0,0,-9.81)
        #p.setGravity(0,0,0)
        # load models
        p.setAdditionalSearchPath("../models")

        self.plane = p.loadURDF("plane/plane.urdf",
                                useFixedBase=True)
        p.changeDynamics(self.plane,-1,restitution=.95)

        self.robot = p.loadURDF("indy_dual_arm/indy_dual_arm.urdf",
                                useFixedBase=True,
                                flags=p.URDF_USE_SELF_COLLISION)
        
        # robot parameters
        self.dof = 12
        


        self.left_joints = [2,3,4,5,6,7]
        self.left_gripper_joints = [10,11,12,13]
        self.right_joints = [15,16,17,18,19,20]        
        self.right_gripper_joints = [23,24,25,26]
        self.joints=[]        
        self.q_min = []
        self.q_max = []
        self.target_pos = []
        self.target_torque = []
        self.left_target_torque =[]
        self.right_target_torque =[]
        self.gripper_control_mode="position" 


        for j in self.left_joints:
            joint_info = p.getJointInfo(self.robot, j)
            self.joints.append(j)
            self.q_min.append(joint_info[8])
            self.q_max.append(joint_info[9])
            self.target_pos.append((self.q_min[len(self.q_min)-1] + self.q_max[len(self.q_max)-1])/2.0)
            self.left_target_torque.append(0.)
            self.target_torque.append(0.)
        
        for j in self.right_joints:
            joint_info = p.getJointInfo(self.robot, j)
            self.joints.append(j)
            self.q_min.append(joint_info[8])
            self.q_max.append(joint_info[9])
            self.target_pos.append((self.q_min[len(self.q_min)-1] + self.q_max[len(self.q_max)-1])/2.0)
            self.right_target_torque.append(0.)
            self.target_torque.append(0.)

       
        self.reset()

    def reset(self):
        self.t = 0.0        
        self.control_mode = "torque"
        for j in range(len(self.left_joints)):
            self.target_pos[j] = (self.q_min[j] + self.q_max[j])/2.0
            self.target_torque[j] = 0.
            p.resetJointState(self.robot,self.left_joints[j],targetValue=self.target_pos[j])
        for j in range(len(self.left_joints)):
            self.target_pos[j+6] = (self.q_min[j+6] + self.q_max[j+6])/2.0
            self.target_torque[j+6] = 0.
            p.resetJointState(self.robot,self.right_joints[j],targetValue=self.target_pos[j+6])
        self.resetController()

    def step(self):
        self.t += self.stepsize
        p.stepSimulation()

    # robot functions
    def resetLeftGripperController(self):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.left_gripper_joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0. for i in range(len(self.left_gripper_joints))])
    def resetRightGripperController(self):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.right_gripper_joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0. for i in range(len(self.right_gripper_joints))])      
    def resetLeftController(self):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.left_joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0. for i in range(len(self.left_joints))])
    def resetRightController(self):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.right_joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0. for i in range(len(self.right_joints))])
    def resetController(self):
        self.resetLeftController();
        self.resetRightController();     
        self.resetLeftGripperController();
        self.resetRightGripperController();   
    def setControlMode(self, mode):
        if mode == "position":
            self.control_mode = "position"
        elif mode == "torque":
            if self.control_mode != "torque":
                self.resetController()
            self.control_mode = "torque"
        else:
            raise Exception('wrong control mode')

    def setRightTargetPositions(self, target_pos):
        self.target_pos = target_pos
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=self.target_pos,
                                    forces=self.max_torque,
                                    positionGains=self.position_control_gain_p,
                                    velocityGains=self.position_control_gain_d)

    def setTargetTorques(self, target_torque):
        self.target_torque = target_torque
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=[2,3,4,5,6,7,10,11,12,13,15,16,17,18,19,20,23,24,25,26],
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=self.target_torque) 
        #self.setLeftTargetTorques(target_torque[0:6])
        #self.setLeftTargetTorques(target_torque[6:12])
    def setLeftTargetTorques(self, left_target_torque):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.left_joints,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=left_target_torque)         
    def setRightTargetTorques(self, right_target_torque):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.right_joints,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=right_target_torque)  
    def setRightGripperState(self, val):
        
        if val==1:
            target =[1.2,1.2,1.2,1.2]
        else:
            target =[0,0,0,0]
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.right_gripper_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=target,
                                    forces=self.gripper_max_torque,
                                    positionGains=self.gripper_position_control_gain_p,
                                    velocityGains=self.gripper_position_control_gain_p) 
    def setLeftGripperState(self, val):
        if val==1:
            target =[1.2,1.2,1.2,1.2]
        else:
            target =[0,0,0,0]        
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.left_gripper_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=target,
                                    forces=self.gripper_max_torque,
                                    positionGains=self.gripper_position_control_gain_p,
                                    velocityGains=self.gripper_position_control_gain_p)  
    def getLeftJointStates(self):
        left_joint_states = p.getJointStates(self.robot, self.left_joints)
        left_joint_pos = [x[0] for x in left_joint_states]
        left_joint_vel = [x[1] for x in left_joint_states]
        return left_joint_pos, left_joint_vel 
    def getRightJointStates(self):
        right_joint_states = p.getJointStates(self.robot, self.right_joints)
        right_joint_pos = [x[0] for x in right_joint_states]
        right_joint_vel = [x[1] for x in right_joint_states]
        return right_joint_pos, right_joint_vel         
    def getJointStates(self):
        #left_joint_pos, left_joint_vel  = self.getLeftJointStates();
        #right_joint_pos, right_joint_vel  = self.getRightJointStates();
        #joint_pos=[]
        #joint_vel=[]
        #for i in range(len(left_joint_pos)):
        #    joint_pos.append(left_joint_pos[i])
        #    joint_vel.append(left_joint_vel[i])
        #for i in range(len(right_joint_pos)):            
        #    joint_pos.append(right_joint_pos[i])
        #    joint_vel.append(right_joint_vel[i])
        left_joint_states = p.getJointStates(self.robot, [2,3,4,5,6,7,10,11,12,13,15,16,17,18,19,20,23,24,25,26])
        joint_pos = [x[0] for x in left_joint_states]
        joint_vel = [x[1] for x in left_joint_states]        
        return joint_pos, joint_vel 

    def solveInverseKinematics(self, pos, ori):
        return list(p.calculateInverseKinematics(self.robot, 7, pos, ori))
    def getGravityandCoriolisTorque(self):
        joint_pos, joint_vel = self.getJointStates()
        joint_acc = [0 for x in joint_pos]
        return list(p.calculateInverseDynamics(self.robot,joint_pos, joint_vel, joint_acc))
    	


if __name__ == "__main__":
    robot = Panda(realtime=1)
    while True:
        pass
