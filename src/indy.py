import pybullet as p
import numpy as np
from modern_robotics import *
def w_p_to_slit(w,p):

    Slist =[];

    for i in range(0,len(w)):

        v = -np.cross(np.array(w[i]),np.array(p[i]))

        wi =np.array(w[i])

        S = [wi[0],wi[1],wi[2],v[0],v[1],v[2]];

        Slist.append(S)

    return np.array(Slist).T


class Indy:
    def __init__(self, timeStep=1e-3, realtime=0):
        self.t = 0.0
        self.timeStep = timeStep
        self.realtime = realtime

        self.control_mode = "torque" 

        self.position_control_gain_p = [0.01,0.01,0.01,0.01,0.01,0.01]
        self.position_control_gain_d = [1.0,1.0,1.0,1.0,1.0,1.0]
        self.max_torque = [3000,3000,3000,3000,3000,3000]
        self.joint_init = [0,-0.2618,-1.5708,0,-1.309,0]
        H1 = 0.3;
        H2 = 0.45;
        H3 = 0.350;
        H4 = 0.228;
        W1 = 0.0035;
        W2 = 0.183;        
        w =[]

        p_ =[]

        #right arm
        base_p = []
        base_w = []
        base_p.append([0,0,0])
        base_p.append([0,0,H1])
        base_p.append([0,0,H1+H2])
        base_p.append([0,-W1,H1+H2+H3])
        base_p.append([0,-W1,H1+H2+H3])
        base_p.append([0,-W1-W2,H1+H2+H3])

        base_w.append([0 ,0,  1]);
        base_w.append([0 ,-1, 0]);
        base_w.append([0 ,-1,  0]);
        base_w.append([0 ,0, 1]);
        base_w.append([0 ,-1, 0]);
        base_w.append([0  ,0, 1]);
        self.M = np.array([[1,0,0,0],[0 ,1 ,0 ,-W1-W2 ],[0 ,0 ,1 ,H1+H2+H3+H4],[0 ,0 ,0 ,1 ]])
        self.Slist = w_p_to_slit(base_w,base_p);
        self.Blist = Adjoint(TransInv(self.M))@self.Slist;

        # connect pybullet
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=30, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.5])

        p.resetSimulation()
        p.setTimeStep(self.timeStep)
        p.setRealTimeSimulation(self.realtime)
        p.setGravity(0,0,-9.81)

        # load models
        p.setAdditionalSearchPath("../models")

        self.plane = p.loadURDF("plane/plane.urdf",
                                useFixedBase=True)
        p.changeDynamics(self.plane,-1,restitution=.95)

        self.robot = p.loadURDF("indy/indy.urdf",
                                useFixedBase=True,
                                flags=p.URDF_USE_SELF_COLLISION)
        
        # robot parameters
        self.dof = p.getNumJoints(self.robot) - 1 # Virtual fixed joint between the flange and last link
        if self.dof != 6:
            raise Exception('wrong urdf file: number of joints is not 6')

        self.joints = []
        self.q_min = []
        self.q_max = []
        self.target_q = []
        self.target_torque = []

        for j in range(self.dof):
            joint_info = p.getJointInfo(self.robot, j)
            self.joints.append(j)
            self.q_min.append(joint_info[8])
            self.q_max.append(joint_info[9])
            self.target_q.append((self.q_min[j] + self.q_max[j])/2.0)
            self.target_torque.append(0.)

        self.reset()

    def reset(self):
        self.t = 0.0        
        self.control_mode = "torque"
        for j in range(self.dof):
            self.target_q[j] =self.joint_init[j]
            self.target_torque[j] = 0.
            p.resetJointState(self.robot,j,targetValue=self.target_q[j])

        self.resetController()

    def step(self):
        self.t += self.timeStep
        p.stepSimulation()

    # robot functions
    def resetController(self):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0. for i in range(self.dof)])

    def setControlMode(self, mode):
        if mode == "position":
            self.control_mode = "position"
        elif mode == "torque":
            if self.control_mode != "torque":
                self.resetController()
            self.control_mode = "torque"
        else:
            raise Exception('wrong control mode')

    def setTargetPositions(self, target_q):
        self.target_q = target_q
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=self.target_q,
                                    forces=self.max_torque,
                                    positionGains=self.position_control_gain_p,
                                    velocityGains=self.position_control_gain_d)

    def setTargetTorques(self, target_torque):
        self.target_torque = target_torque
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=self.target_torque)
    def getJacobianMatrix(self,q):
        #result = p.getLinkState(self.robot,self.dof,computeLinkVelocity=1,computeForwardKinematics=1)
        #link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        #zero_vec = [0.0] * len(q)
        #J = p.calculateJacobian(self.robot, self.dof, com_trn,q,zero_vec,zero_vec)
        #linear_J,angular_J = J
        #J = np.concatenate((np.array(angular_J),np.array(linear_J)),axis=0)
        J = JacobianBody(self.Blist,np.array(q))

        return J

    def getJointStates(self):
        joint_states = p.getJointStates(self.robot, self.joints)
        q = [x[0] for x in joint_states]
        qdot = [x[1] for x in joint_states]
        return q, qdot 

    def solveInverseDynamics(self, q, qdot, qddot):
        return list(p.calculateInverseDynamics(self.robot, q, qdot, qddot))

    def solveInverseKinematics(self, pos, ori):
        return np.array(p.calculateInverseKinematics(self.robot, 7, pos, ori))
    def getMassMatrix(self,q):
        M = np.array(p.calculateMassMatrix(self.robot, q, flags=1))
        M = M[0:self.dof,0:self.dof]
        return np.array(M)
    def computeTaskErr(self,posDesired,velDesired):
        result = p.getLinkState(self.robot,self.dof,computeLinkVelocity=1,computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
       # print(link_vt)
        pos = [link_trn[0],link_trn[1],link_trn[2]]
        vel = [link_vt[0],link_vt[1],link_vt[2]]
        e = np.array(posDesired)-np.array(pos)
        edot = np.array(velDesired)-np.array(vel)
        return e, edot
    def getFK(self,q):
        #result = p.getLinkState(self.robot,self.dof,computeLinkVelocity=1,computeForwardKinematics=1)
        #link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        #pos = link_trn
        #R = np.reshape(np.array(p.getMatrixFromQuaternion(link_rot)),(3,3));
        #T = np.eye(4);
        #T[0:3,0:3] = R
        #T[0,3] = pos[0]
        ##T[1,3] = pos[1]
        #T[2,3] = pos[2]
        T = FKinBody(self.M,self.Blist,np.array(q));
      #  print(T)
        return T



if __name__ == "__main__":
    robot = Panda(realtime=1)
    while True:
        pass
